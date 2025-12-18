import pytest
from fastapi.testclient import TestClient
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from src.server import app

client = TestClient(app)

def test_anchor_flow():
    # 1. Define Anchor on P1
    p1 = {
        "context": {"sessionId": "s1"},
        "args": {
            "part": "p1",
            "anchor_id": "a1",
            "origin": [0,0,0],
            "z_axis": [0,0,1],
            "x_axis": [1,0,0]
        }
    }
    resp = client.post("/tools/define_anchor", json=p1)
    assert resp.status_code == 200
    assert resp.json()["result"]["ok"] is True
    
    # 2. Define Anchor on P2
    p2 = {
        "context": {"sessionId": "s1"},
        "args": {
            "part": "p2",
            "anchor_id": "a2",
            "origin": [10,0,0],
            "z_axis": [0,0,1],
            "x_axis": [1,0,0]
        }
    }
    client.post("/tools/define_anchor", json=p2)
    
    # 3. Connect check
    j1 = {
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
    resp = client.post("/tools/define_joint", json=j1)
    assert resp.status_code == 200
    assert resp.json()["result"]["ok"] is True
