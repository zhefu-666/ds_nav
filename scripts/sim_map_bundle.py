#!/usr/bin/env python3
"""Save a bounded, validity-gated local map bundle; no truth or control interface.

ROS imports are delayed so cell/voxel accumulation can be tested offline.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import time
import numpy as np
import yaml


class MapAccumulator:
    def __init__(self, voxel=.05, max_cells=2_000_000, max_points=2_000_000, grid_epoch_validated=False, min_points=100):
        self.voxel, self.max_cells, self.max_points = voxel, max_cells, max_points
        self.grid_epoch_validated, self.min_points = grid_epoch_validated, min_points
        self.points, self.cells = {}, {}
        self.resolution, self.anchor = None, None
        self.valid, self.valid_since = False, None
        self.measurement_ranges={'cloud':None,'grid':None}
        self.windows, self.invalid_reasons = [], []
        self.last_clock, self.last_valid_wall = None, -math.inf
        self.counts = {'accepted_clouds':0, 'accepted_grids':0, 'rejected_clouds':0, 'rejected_grids':0}

    def clock(self, stamp):
        if self.last_clock is not None and stamp < self.last_clock:
            self.invalidate('clock_rollback')
        self.last_clock = stamp

    def invalidate(self, reason):
        if reason not in self.invalid_reasons:
            self.invalid_reasons.append(reason)

    def state(self, valid, stamp, wall):
        if valid:
            self.last_valid_wall = wall
            if not self.valid:
                self.valid_since = stamp
                self.windows.append({'start_ns':stamp, 'end_ns':None})
        elif self.valid:
            self.windows[-1]['end_ns'] = stamp
            self.invalidate('tracking_validity_lost_after_collection_started')
        self.valid = valid

    def eligible(self, stamp, wall):
        if self.valid and wall-self.last_valid_wall > .5:
            self.invalidate('validity_stream_stale_after_collection_started')
        return (self.valid and not self.invalid_reasons and stamp > 0 and
                self.valid_since is not None and stamp >= self.valid_since and
                wall-self.last_valid_wall <= .5)

    def cloud(self, points, stamp, frame, wall):
        if frame != 'camera_init' or not self.eligible(stamp,wall):
            self.counts['rejected_clouds'] += 1; return
        points = np.asarray(points,dtype=float).reshape(-1,3)
        points = points[np.isfinite(points).all(1)]
        keys = np.floor(points/self.voxel).astype(np.int64)
        for key, point in zip(keys,points):
            self.points.setdefault(tuple(key),point.tolist())
        self.counts['accepted_clouds'] += 1
        previous=self.measurement_ranges['cloud']
        self.measurement_ranges['cloud']=[min(previous[0],stamp),max(previous[1],stamp)] if previous else [stamp,stamp]
        if len(self.points)>self.max_points:
            self.invalidate('point_budget_exceeded')

    def grid(self, values, width, height, resolution, origin, stamp, frame, wall):
        if frame != 'camera_init' or not self.eligible(stamp,wall):
            self.counts['rejected_grids'] += 1; return
        if not math.isfinite(resolution) or resolution<=0:
            self.invalidate('invalid_grid_resolution'); return
        if self.resolution is None:
            self.resolution = resolution
            self.anchor = np.asarray(origin,dtype=float)
        delta = (np.asarray(origin)-self.anchor)/self.resolution
        if abs(resolution-self.resolution)>1e-6 or not np.allclose(delta,np.round(delta),atol=1e-4):
            self.invalidate('sliding_grid_lattice_changed'); return
        values=np.asarray(values).reshape(height,width)
        if not np.isin(values,[0,33,66,100]).all():
            self.invalidate('unexpected_layer_type_encoding'); return
        offset=np.round(delta).astype(int)
        for row,col in np.argwhere(values!=0):
            # Unknown never overwrites a cell observed in an earlier window.
            self.cells[(int(col+offset[0]),int(row+offset[1]))] = int(values[row,col])
        self.counts['accepted_grids'] += 1
        previous=self.measurement_ranges['grid']
        self.measurement_ranges['grid']=[min(previous[0],stamp),max(previous[1],stamp)] if previous else [stamp,stamp]
        if len(self.cells)>self.max_cells:
            self.invalidate('cell_budget_exceeded')

    def finish(self, directory, seed, duration, reason):
        if len(self.points)<self.min_points: self.invalidate('insufficient_valid_points')
        if not self.grid_epoch_validated: self.invalidate('grid_initialization_provenance_unverified')
        if not self.cells: self.invalidate('no_valid_observed_cells')
        if self.valid and self.last_clock is not None:
            self.windows[-1]['end_ns']=max(self.last_clock,self.windows[-1]['start_ns'])
        manifest={'schema_version':1, 'status':'INVALID' if self.invalid_reasons else 'VALID_MAP_BUNDLE',
                  'invalid_reasons':self.invalid_reasons, 'frame':'camera_init', 'source_frame':'camera_init',
                  'map_gauge':'local_initial_lio', 'reload_frame_name':'map',
                  'validation':'STRUCTURAL_ONLY_NOT_NAVIGATION_ACCEPTANCE',
                  'grid_epoch_provenance':{'started_after_valid_asserted_by_caller':self.grid_epoch_validated,
                                           'note':'layer_type cannot reveal historical pre-valid input cells; parent must start ROGMap after valid'},
                  'relocalization_initial_pose':seed,
                  'gauge':'saved first-session camera_init renamed map; no world/truth alignment',
                  'truth_used':False, 'world_transform_available':False,
                  'reload_initial_pose':{'values':seed,'convention':'saved_map <- new_camera_init; xyz metres then roll pitch yaw radians',
                                         'source':'explicit CLI configuration; never inferred from truth'},
                  'valid_windows':self.windows, 'counts':self.counts,
                  'accepted_measurement_range_ns':self.measurement_ranges,
                  'point_count':len(self.points), 'observed_cell_count':len(self.cells),
                  'voxel_resolution_m':self.voxel, 'grid_resolution_m':self.resolution,
                  'cell_policy':'latest observed type wins; UNKNOWN never clears historical observations',
                  'type_encoding':{'0':'UNKNOWN','33':'FREE','66':'PASSABLE','100':'OCCUPIED'},
                  'duration_wall_s':duration, 'stop_reason':reason, 'files':{}}
        # Invalid collections retain the manifest only, never a loadable partial map.
        if not self.invalid_reasons:
            pts=np.asarray(list(self.points.values()),dtype='<f4')
            pcd=directory/'map.pcd'
            header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n'
            pcd.write_bytes(header.encode()+pts.tobytes())
            coords=np.asarray(list(self.cells));low,high=coords.min(0),coords.max(0)
            size=high-low+1
            if int(size[0])*int(size[1])>self.max_cells:
                self.invalidate('export_rectangle_budget_exceeded')
                pcd.unlink()
                manifest.update(status='INVALID',invalid_reasons=self.invalid_reasons)
            else:
                image=np.full((size[1],size[0]),205,dtype=np.uint8)
                for (x,y),kind in self.cells.items():image[y-low[1],x-low[0]]=0 if kind==100 else 254
                pgm=directory/'map.pgm';pgm.write_bytes(f'P5\n{size[0]} {size[1]}\n255\n'.encode()+np.flipud(image).tobytes())
                origin=(self.anchor+low*self.resolution).tolist()
                map_yaml={'image':'map.pgm','resolution':self.resolution,'origin':[origin[0],origin[1],0.],
                          'negate':0,'occupied_thresh':.65,'free_thresh':.196,'mode':'trinary'}
                (directory/'map.yaml').write_text(yaml.safe_dump(map_yaml,sort_keys=False))
                for key,name in [('pcd','map.pcd'),('yaml','map.yaml'),('image','map.pgm')]:
                    manifest['files'][key]={'path':name,'sha256':hashlib.sha256((directory/name).read_bytes()).hexdigest()}
                manifest['grid_size']={'width':int(size[0]),'height':int(size[1]),'origin':origin,
                                       'unknown_cells':int((image==205).sum())}
        (directory/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
        return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--duration',type=float,required=True,help='maximum wall seconds')
    parser.add_argument('--reload-seed',nargs=6,type=float,required=True,metavar=('X','Y','Z','ROLL','PITCH','YAW'))
    parser.add_argument('--grid-started-after-valid',action='store_true',
                        help='assert parent started fresh ROGMap only after mapping-valid; otherwise bundle INVALID')
    parser.add_argument('--min-points',type=int,default=100)
    parser.add_argument('--voxel',type=float,default=.05)
    parser.add_argument('--max-cells',type=int,default=2_000_000)
    parser.add_argument('--max-points',type=int,default=2_000_000)
    args=parser.parse_args()
    if not 0<args.duration<=86400 or not math.isfinite(args.voxel) or args.voxel<=0 or min(args.max_cells,args.max_points,args.min_points)<=0 or not all(math.isfinite(x) for x in args.reload_seed):
        parser.error('finite seed, positive bounds and duration <=86400 required')
    if args.out.exists():parser.error('output directory already exists')
    args.out.mkdir(parents=True)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from sensor_msgs_py import point_cloud2
    rclpy.init()
    def terminate(_signal,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    node=Node('sim_local_map_bundle',parameter_overrides=[Parameter('use_sim_time',value=True)])
    accumulator=MapAccumulator(args.voxel,args.max_cells,args.max_points,args.grid_started_after_valid,args.min_points)
    def ns(stamp):return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
    node.create_subscription(Clock,'/clock',lambda m:accumulator.clock(ns(m.clock)),qos_profile_sensor_data)
    node.create_subscription(Bool,'/sim/localization_valid',lambda m:accumulator.state(m.data,node.get_clock().now().nanoseconds,time.monotonic()),10)
    def cloud(msg):
        wall=time.monotonic()
        # Drop invalid windows before decoding the full point payload.
        if not accumulator.eligible(ns(msg.header.stamp),wall):
            accumulator.counts['rejected_clouds']+=1;return
        points=np.asarray([tuple(p) for p in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)])
        accumulator.cloud(points,ns(msg.header.stamp),msg.header.frame_id,wall)
    def grid(msg):
        q=msg.info.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z)>1e-5 or abs(abs(q.w)-1)>1e-5:
            accumulator.invalidate('rotated_grid_origin_unsupported');return
        accumulator.grid(msg.data,msg.info.width,msg.info.height,msg.info.resolution,
                         [msg.info.origin.position.x,msg.info.origin.position.y],ns(msg.header.stamp),msg.header.frame_id,time.monotonic())
    node.create_subscription(PointCloud2,'/cloud_registered_full',cloud,qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid,'/rog_map/layer_type',grid,qos_profile_sensor_data)
    started=time.monotonic();reason='duration_reached'
    try:
        while rclpy.ok() and time.monotonic()-started<args.duration:
            rclpy.spin_once(node,timeout_sec=.05)
            if accumulator.valid and time.monotonic()-accumulator.last_valid_wall>.5:
                accumulator.invalidate('validity_stream_stale_after_collection_started')
            if accumulator.invalid_reasons:
                reason='invalid_collection';break
    except (KeyboardInterrupt,ExternalShutdownException):reason='interrupted'
    finally:
        manifest=accumulator.finish(args.out,args.reload_seed,time.monotonic()-started,reason)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(manifest,indent=2))
    return 0 if manifest['status']=='VALID_MAP_BUNDLE' else 2


if __name__=='__main__':raise SystemExit(main())
