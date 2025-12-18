import pytest
from fastapi.testclient import TestClient
from unittest.mock import MagicMock, patch
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

with patch.dict(sys.modules, {'fcl': MagicMock(), 'trimesh': MagicMock(), 'matplotlib': MagicMock(), 'matplotlib.pyplot': MagicMock(), 'mpl_toolkits.mplot3d': MagicMock(), 'networkx': MagicMock()}):
    from src.server import app, engine

client = TestClient(app)

def test_define_anchor():
    with patch.object(engine, 'define_anchor') as mock_anchor:
        mock_anchor.return_value = {"ok": True}
        payload = {
            "context": {"sessionId": "s1"},
            "args": {
                "part": "p1",
                "anchor_id": "a1",
                "origin": [0,0,0],
                "z_axis": [0,0,1],
                "x_axis": [1,0,0]
            }
        }
        resp = client.post("/tools/define_anchor", json=payload)
        assert resp.status_code == 200
        assert resp.json()["result"]["ok"] is True

def test_define_joint():
    with patch.object(engine, 'define_joint') as mock_joint:
        mock_joint.return_value = {"ok": True}
        payload = {
            "context": {"sessionId": "s1"},
            "args": {
                "joint_id": "j1",
                "type": "revolute",
                "parent_part": "p1",
                "parent_anchor": "a1",
                "child_part": "p2",
                "child_anchor": "a2",
                "limits": [0, 90]
            }
        }
        resp = client.post("/tools/define_joint", json=payload)
        assert resp.status_code == 200
