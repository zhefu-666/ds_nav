#!/usr/bin/env python3
"""Offline map lifecycle, validity-window and artifact tests; no ROS needed."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from sim_map_bundle import MapAccumulator


class MapBundleTests(unittest.TestCase):
    def ready(self, **kwargs):
        m=MapAccumulator(grid_epoch_validated=True,min_points=1,**kwargs)
        m.clock(1_000_000_000);m.state(True,1_000_000_000,10.)
        return m

    def populate(self,m):
        m.cloud([[0.,0.,0.],[.1,0.,0.]],1_100_000_000,'camera_init',10.1)
        m.grid([33,0,100,66],2,2,.05,[0.,0.],1_100_000_000,'camera_init',10.1)

    def test_invalid_initial_cloud_is_excluded(self):
        m=self.ready()
        m.cloud([[99.,99.,99.]],900_000_000,'camera_init',10.1)
        self.populate(m)
        self.assertEqual(len(m.points),2)
        self.assertEqual(m.counts['rejected_clouds'],1)

    def test_sliding_accumulation_unknown_does_not_overwrite(self):
        m=self.ready();self.populate(m)
        m.grid([0,100,0,0],2,2,.05,[.05,0.],1_200_000_000,'camera_init',10.2)
        self.assertEqual(m.cells[(0,0)],33)
        self.assertEqual(m.cells[(0,1)],100)
        self.assertEqual(m.cells[(2,0)],100)
        self.assertNotIn((1,0),m.cells)

    def test_artifacts_unknown_origin_and_hashes(self):
        m=self.ready();self.populate(m)
        with tempfile.TemporaryDirectory() as d:
            directory=Path(d)
            out=m.finish(directory,[0.]*6,1.,'test')
            self.assertEqual(out['status'],'VALID_MAP_BUNDLE')
            self.assertEqual(out['frame'],'camera_init')
            self.assertEqual(out['map_gauge'],'local_initial_lio')
            for spec in out['files'].values():
                self.assertEqual(hashlib.sha256((directory/spec['path']).read_bytes()).hexdigest(),spec['sha256'])
            raw=(directory/'map.pgm').read_bytes().split(b'255\n',1)[1]
            np.testing.assert_array_equal(np.frombuffer(raw,dtype=np.uint8).reshape(2,2),[[0,254],[254,205]])
            self.assertEqual(json.loads((directory/'manifest.json').read_text())['files'],out['files'])

    def test_tracking_loss_has_manifest_only(self):
        m=self.ready();self.populate(m);m.state(False,1_200_000_000,10.2)
        m.state(True,1_300_000_000,10.3)
        with tempfile.TemporaryDirectory() as d:
            out=m.finish(Path(d),[0.]*6,1.,'test')
            self.assertEqual(out['status'],'INVALID')
            self.assertEqual(list(Path(d).iterdir()),[Path(d)/'manifest.json'])

    def test_clock_rollback_and_stale_validity_fail(self):
        m=self.ready();self.populate(m);m.clock(10)
        self.assertIn('clock_rollback',m.invalid_reasons)
        m=self.ready();m.eligible(1_200_000_000,11.)
        self.assertIn('validity_stream_stale_after_collection_started',m.invalid_reasons)

    def test_resolution_and_lattice_changes_fail(self):
        m=self.ready();self.populate(m)
        m.grid([33],1,1,.05,[.023,0.],1_200_000_000,'camera_init',10.2)
        self.assertIn('sliding_grid_lattice_changed',m.invalid_reasons)

    def test_grid_provenance_is_required(self):
        m=MapAccumulator(min_points=1)
        m.state(True,1_000_000_000,10.);self.populate(m)
        with tempfile.TemporaryDirectory() as d:
            out=m.finish(Path(d),[0.]*6,1.,'test')
            self.assertIn('grid_initialization_provenance_unverified',out['invalid_reasons'])
            self.assertFalse(out['files'])


if __name__=='__main__':unittest.main()
