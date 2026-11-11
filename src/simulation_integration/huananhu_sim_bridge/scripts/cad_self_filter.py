"""Measured-joint, link-local CAD hull surface hit filter. Never fills rays."""
import bisect
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation


class MeasuredJointHistory:
    def __init__(self, required, max_gap=.1):
        self.required = set(required)
        self.samples = []
        self.max_gap = max_gap

    def append(self, stamp, names, positions):
        q = dict(zip(names, positions))
        if not self.required <= q.keys():
            raise ValueError('missing_measured_joints:' + ','.join(sorted(self.required-q.keys())))
        q = {k:float(q[k]) for k in self.required}
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in q.values()):
            raise ValueError('nonfinite_joint_sample')
        if self.samples and stamp <= self.samples[-1][0]:
            raise ValueError('nonmonotonic_joint_history')
        self.samples.append((stamp,q))
        self.samples = [s for s in self.samples if s[0] >= stamp-3]

    def at(self, stamp):
        i = bisect.bisect_left([s[0] for s in self.samples],stamp)
        if i == 0 or i == len(self.samples):
            raise ValueError('joint_history_not_bracketed')
        a,b=self.samples[i-1],self.samples[i]
        if b[0]-a[0] > self.max_gap:
            raise ValueError('joint_history_gap')
        ratio=(stamp-a[0])/(b[0]-a[0])
        return {k:a[1][k]+ratio*(b[1][k]-a[1][k]) for k in self.required}


def read_stl_triangles(path):
    """Read source STL without optional trimesh/rtree dependencies."""
    raw = Path(path).read_bytes()
    if len(raw) >= 84:
        count = int.from_bytes(raw[80:84], 'little')
        if len(raw) == 84 + count * 50:
            dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
            return np.frombuffer(raw, dtype=dtype, offset=84, count=count)['vertices'].astype(float)
    vertices = [list(map(float,line.split()[1:4])) for line in raw.decode('ascii').splitlines()
                if line.lstrip().startswith('vertex ')]
    return np.asarray(vertices, dtype=float).reshape(-1,3,3)


def closest_triangle_points(triangles, points):
    """Exact closest points using plane projection and clamped edge segments."""
    a,b,c = triangles[:,0],triangles[:,1],triangles[:,2]
    ab,ac = b-a,c-a
    normal = np.cross(ab,ac)
    normal_sq = np.einsum('ij,ij->i',normal,normal)
    offset = np.einsum('ij,ij->i',points-a,normal) / np.maximum(normal_sq,1e-30)
    projected = points-normal*offset[:,None]
    ap = projected-a
    d00=np.einsum('ij,ij->i',ab,ab);d01=np.einsum('ij,ij->i',ab,ac)
    d11=np.einsum('ij,ij->i',ac,ac);d20=np.einsum('ij,ij->i',ap,ab);d21=np.einsum('ij,ij->i',ap,ac)
    denominator=d00*d11-d01*d01
    u=(d11*d20-d01*d21)/np.maximum(denominator,1e-30)
    v=(d00*d21-d01*d20)/np.maximum(denominator,1e-30)
    inside=(u>=0)&(v>=0)&(u+v<=1)&(normal_sq>1e-25)
    best=np.full(len(points),np.inf)
    for start,end in ((a,b),(b,c),(c,a)):
        edge=end-start
        t=np.clip(np.einsum('ij,ij->i',points-start,edge)/np.maximum(np.einsum('ij,ij->i',edge,edge),1e-30),0,1)
        delta=points-start-t[:,None]*edge
        best=np.minimum(best,np.einsum('ij,ij->i',delta,delta))
    delta=points-projected
    best[inside]=np.einsum('ij,ij->i',delta[inside],delta[inside])
    return np.sqrt(best)


class CadSelfFilter:
    def __init__(self, resource_dir, tolerance=.001, geometry="visual_surface"):
        if tolerance <= 0 or tolerance > .005:
            raise ValueError('surface tolerance must be (0,.005] metres')
        if geometry not in ('visual_surface', 'hull_surface'):
            raise ValueError('unsupported self-filter geometry')
        self.geometry=geometry
        self.tolerance=tolerance
        share=Path(resource_dir)
        self.joints={}
        root=ET.parse(share/'urdf/huananhu.urdf').getroot()
        for j in root.findall('joint'):
            origin=j.find('origin');matrix=np.eye(4)
            if origin is not None:
                matrix[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                matrix[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            axis=j.find('axis')
            self.joints[j.find('child').get('link')]={
                'parent':j.find('parent').get('link'),'name':j.get('name'),
                'type':j.get('type'),'origin':matrix,
                'axis':np.fromstring(axis.get('xyz'),sep=' ') if axis is not None else np.array([1.,0.,0.])}
        self.hulls={}
        for path in sorted((share/'models/huananhu_demo/collision').glob('*_hull.obj')):
            vertices=np.array([[float(x) for x in line.split()[1:4]] for line in path.read_text().splitlines() if line.startswith('v ')])
            hull=ConvexHull(vertices)
            # Coplanar tessellation creates redundant equations; dedup once.
            eq=np.unique(np.round(hull.equations,10),axis=0)
            self.hulls[path.name[:-9]]=(eq,vertices.min(axis=0),vertices.max(axis=0))
        if not self.hulls or 'yaw1' not in self.hulls:
            raise ValueError('missing_CAD_hulls')
        # Wheels render real self returns too; use their measured joint angles.
        self.links = set(self.hulls)
        if geometry == 'visual_surface':
            self.links.update(link.get('name') for link in root.findall('link')
                              if link.get('name', '').startswith('wheel_'))
        self.required=set()
        for link in self.links:
            while link != 'base_link':
                j=self.joints[link]
                if j['type']!='fixed':self.required.add(j['name'])
                link=j['parent']
        self.sensor=np.eye(4)
        self.sensor[:3,3]=[.0557597190051302,.106643863403111,.163594690492484]
        self.sensor[:3,:3]=Rotation.from_euler('x',-math.pi/2).as_matrix()

        self.visuals = {}
        self.visual_trees = {}
        self.visual_bounds = {}
        if geometry == 'visual_surface':
            mesh_cache = {}
            for link in root.findall('link'):
                name = link.get('name')
                # Only remove returns on a rendered surface at measured q.
                if name not in self.links:
                    continue
                parts=[]
                for visual in link.findall('visual'):
                    mesh=visual.find('geometry/mesh')
                    if mesh is None:
                        continue
                    path=share/'models/huananhu_demo/meshes'/mesh.get('filename').split('/')[-1]
                    if path not in mesh_cache:
                        mesh_cache[path]=read_stl_triangles(path)
                    triangles=mesh_cache[path]
                    origin=visual.find('origin')
                    if origin is not None:
                        rotation=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                        translation=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                        triangles=triangles@rotation.T+translation
                    parts.append(triangles)
                if parts:
                    triangles=np.concatenate(parts)
                    self.visuals[name]=(triangles,cKDTree(triangles.mean(1)),triangles.min((0,1)),triangles.max((0,1)))
                    self.visual_bounds[name] = (triangles.min(1), triangles.max(1))
                    radius = np.linalg.norm(triangles-triangles.mean(1)[:,None,:],axis=2).max(axis=1)
                    bins = np.ceil(np.log2(np.maximum(radius,1e-12))).astype(int)
                    self.visual_trees[name] = []
                    for bucket in np.unique(bins):
                        ids = np.flatnonzero(bins == bucket)
                        self.visual_trees[name].append((float(radius[ids].max()), ids,
                            cKDTree(triangles[ids].mean(1))))


    def poses(self, positions):
        if not self.required <= positions.keys():raise ValueError('missing_measured_joints')
        poses={'base_link':np.eye(4)}
        def visit(link):
            if link not in poses:
                j=self.joints[link];motion=np.eye(4)
                if j['type']=='prismatic':motion[:3,3]=j['axis']*positions[j['name']]
                elif j['type']!='fixed':motion[:3,:3]=Rotation.from_rotvec(j['axis']*positions[j['name']]).as_matrix()
                poses[link]=visit(j['parent'])@j['origin']@motion
            return poses[link]
        for link in self.links:visit(link)
        return poses

    def classify(self, xyz, positions, ray_ids=None):
        poses=self.poses(positions);sensor_pose=poses['yaw1']@self.sensor
        hits=np.zeros(len(xyz),dtype=bool);counts={}
        if getattr(self, 'geometry', 'hull_surface') == 'visual_surface':
            if ray_ids is not None:
                ray_ids=np.asarray(ray_ids,dtype=np.int64)
                if ray_ids.shape != (len(xyz),) or np.any(ray_ids<0):
                    raise ValueError('invalid_original_ray_slots')
            if not hasattr(self, 'triangle_cache'):
                self.triangle_cache={}
            for link,(triangles,tree,lo,hi) in self.visuals.items():
                transform=np.linalg.inv(poses[link])@sensor_pose
                local=xyz@transform[:3,:3].T+transform[:3,3]
                candidates=np.flatnonzero(np.all((local>=lo-self.tolerance)&(local<=hi+self.tolerance),axis=1))
                found=[]
                cache=self.triangle_cache.setdefault(link,{}) if ray_ids is not None else None
                if cache is not None and len(candidates):
                    previous=np.array([cache.get(int(ray_ids[i]),-1) for i in candidates],dtype=np.int64)
                    cached_mask=previous>=0
                    cached_indices=candidates[cached_mask]
                    if len(cached_indices):
                        # Cache contains triangle IDs only: every new point is
                        # transformed with CURRENT measured q and revalidated.
                        distance=closest_triangle_points(triangles[previous[cached_mask]],local[cached_indices])
                        confirmed=cached_indices[distance<=self.tolerance]
                        found.extend(confirmed)
                        keep=np.ones(len(candidates),dtype=bool)
                        keep[np.flatnonzero(cached_mask)[distance<=self.tolerance]]=False
                        candidates=candidates[keep]
                # Candidate method is conservative: missed triangles retain
                # points; only exact surface-proximity positives are removed.
                for chunk in np.array_split(candidates,max(1,math.ceil(len(candidates)/64))):
                    if len(chunk):
                        k=min(128,len(triangles))
                        _,ids=tree.query(local[chunk],k=k)
                        ids=np.asarray(ids).reshape(-1,k)
                        distance=closest_triangle_points(triangles[ids.reshape(-1)],np.repeat(local[chunk],k,axis=0)).reshape(-1,k)
                        nearest=distance.argmin(axis=1)
                        positive=distance[np.arange(len(chunk)),nearest]<=self.tolerance
                        found.extend(chunk[positive])
                        if cache is not None:
                            for index,triangle in zip(chunk[positive],ids[np.arange(len(chunk)),nearest][positive]):
                                cache[int(ray_ids[index])]=int(triangle)
                        # A long face can contain the hit while its centroid is
                        # outside the nearest 128. A bounding-sphere fallback is
                        # complete; removal still requires exact surface distance.
                        for index in chunk[~positive]:
                            candidates_ids = []
                            for radius, triangle_ids, sphere_tree in self.visual_trees[link]:
                                selected = sphere_tree.query_ball_point(local[index], radius+self.tolerance)
                                candidates_ids.extend(triangle_ids[selected])
                            if not candidates_ids:
                                continue
                            ids_array = np.asarray(candidates_ids, dtype=np.int64)
                            lower, upper = self.visual_bounds[link]
                            in_box = np.all((local[index] >= lower[ids_array]-self.tolerance) &
                                            (local[index] <= upper[ids_array]+self.tolerance), axis=1)
                            candidates_ids = ids_array[in_box]
                            if len(candidates_ids) == 0:
                                continue
                            candidate_distance = closest_triangle_points(
                                triangles[candidates_ids], np.broadcast_to(local[index], (len(candidates_ids),3)))
                            best = int(np.argmin(candidate_distance))
                            if candidate_distance[best] <= self.tolerance:
                                found.append(index)
                                if cache is not None:
                                    cache[int(ray_ids[index])] = candidates_ids[best]
                hits[found]=True;counts[link]=len(found)
            return hits,counts
        for link,(eq,lo,hi) in self.hulls.items():
            transform=np.linalg.inv(poses[link])@sensor_pose
            local=xyz@transform[:3,:3].T+transform[:3,3]
            candidates=np.flatnonzero(np.all((local>=lo-self.tolerance)&(local<=hi+self.tolerance),axis=1))
            # Only the convex surface band is supported evidence. Deep interior
            # or concavity hits are retained rather than assumed to be self.
            found=[]
            for chunk in np.array_split(candidates,max(1,math.ceil(len(candidates)/128))):
                if len(chunk):
                    signed=(local[chunk]@eq[:,:3].T+eq[:,3]).max(axis=1)
                    found.extend(chunk[np.abs(signed)<=self.tolerance])
            hits[found]=True;counts[link]=len(found)
        return hits,counts
