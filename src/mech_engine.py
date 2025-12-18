import trimesh
import fcl
import numpy as np
import os
import requests
import io
import base64
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Dict, List, Optional, Tuple, Any
import networkx as nx

class MechEngine:
    def __init__(self, api_url: str):
        self.api_url = api_url
        # anchors[part_name][anchor_id] = { matrix: 4x4, type: str, origin: vec3, z_axis: vec3 }
        self.anchors: Dict[str, Dict[str, Any]] = {} 
        # joints[joint_id] = { type, parent_part, parent_anchor, child_part, child_anchor, limits }
        self.joints: Dict[str, Dict] = {}
        
        self.parts_cache: Dict[str, Any] = {}
        self.session_cache: Dict[str, Dict[str, Any]] = {}

    def define_anchor(self, part: str, anchor_id: str, origin: List[float], 
                     z_axis: List[float], x_axis: List[float]) -> Dict[str, Any]:
        """Define a local coordinate frame on a part."""
        if part not in self.anchors:
            self.anchors[part] = {}
            
        # Construct Matrix from axes
        # Z = primary (Hinge Axis)
        # X = secondary (Orientation)
        # Y = Z cross X
        Z = np.array(z_axis)
        Z = Z / np.linalg.norm(Z)
        
        X = np.array(x_axis)
        X = X / np.linalg.norm(X)
        
        # Ensure orthogonality
        Y = np.cross(Z, X)
        if np.linalg.norm(Y) < 0.001:
            # Fallback if X is parallel to Z
             raise ValueError("Z and X axes cannot be parallel")
        Y = Y / np.linalg.norm(Y)
        X = np.cross(Y, Z) # Recompute X to be perfectly orthogonal
        
        # 4x4 Matrix
        # [ Xx Yx Zx Tx ]
        # [ Xy Yy Zy Ty ]
        # [ Xz Yz Zz Tz ]
        # [ 0  0  0  1  ]
        M = np.eye(4)
        M[:3, 0] = X
        M[:3, 1] = Y
        M[:3, 2] = Z
        M[:3, 3] = np.array(origin)
        
        self.anchors[part][anchor_id] = {
            "matrix": M,
            "origin": origin,
            "z_axis": Z.tolist(),
            "x_axis": X.tolist()
        }
        
        # Visual Debug
        try:
            debug_image_base64 = self._render_anchor_debug(origin, Z, X)
            debug_image_url = f"data:image/png;base64,{debug_image_base64}"
        except Exception as e:
            print(f"Failed to render debug image: {e}")
            debug_image_url = ""
            
        return {
            "ok": True,
            "anchor": {
                "part": part,
                "id": anchor_id,
                "origin": origin
            },
            "debug_image": debug_image_url
        }

    def define_joint(self, joint_id: str, type: str, parent_part: str, parent_anchor: str,
                    child_part: str, child_anchor: str, limits: List[float]) -> Dict[str, Any]:
        """Connect two parts via anchors."""
        
        # Validate existence
        if parent_part not in self.anchors or parent_anchor not in self.anchors[parent_part]:
             return {"ok": False, "error": f"Anchor {parent_part}:{parent_anchor} not found"}
        if child_part not in self.anchors or child_anchor not in self.anchors[child_part]:
             return {"ok": False, "error": f"Anchor {child_part}:{child_anchor} not found"}

        self.joints[joint_id] = {
            "type": type,
            "parent_part": parent_part,
            "parent_anchor": parent_anchor,
            "child_part": child_part,
            "child_anchor": child_anchor,
            "limits": limits
        }
        
        return {"ok": True, "joint_id": joint_id}

    def motion_sweep_check(self, session_id: str, joint_ids: List[str], step_deg: float = 5.0) -> Dict[str, Any]:
        """Check collisions by assembling parts and sweeping the target joint."""
        if not joint_ids:
            return {"ok": False, "error": "No joints specified"}
        
        target_joint_id = joint_ids[0]
        target_joint = self.joints.get(target_joint_id)
        if not target_joint:
            return {"ok": False, "error": f"Joint {target_joint_id} not found"}

        # Build Assembly Graph
        # Nodes = Parts, Edges = Joints
        G = nx.DiGraph()
        all_parts = set()
        for jid, j in self.joints.items():
            G.add_edge(j["parent_part"], j["child_part"], joint=j)
            all_parts.add(j["parent_part"])
            all_parts.add(j["child_part"])
            
        # Determine Root (simplistic: standard "parent" of target joint, or first node with 0 in-degree)
        root_part = target_joint["parent_part"] 
        # ideally walk up the tree
        curr = root_part
        while True:
            preds = list(G.predecessors(curr))
            if not preds:
                break
            curr = preds[0]
        root_part = curr

        # Load Meshes
        meshes = {}
        for part in all_parts:
            m = self.fetch_artifact(session_id, part)
            if m:
                if isinstance(m, trimesh.Scene):
                     m = trimesh.util.concatenate(m.dump(concatenate=True))
                meshes[part] = m
            else:
                return {"ok": False, "error": f"Failed to load mesh {part}"}

        # Sweep Logic
        lim_min, lim_max = target_joint["limits"]
        steps = np.arange(lim_min, lim_max + step_deg, step_deg)
        collisions = []

        collision_manager = trimesh.collision.CollisionManager()

        for angle in steps:
            # Update Transforms for the graph
            # For each edge, T_child_local = T_joint_motion * T_alignment
            
            # Logic:
            # 1. T_parent (World)
            # 2. T_anchor_parent (Local)
            # 3. T_joint (Rotation)
            # 4. T_anchor_child_inv (Local Inv) -> brings child anchor to origin
            
            # T_child (World) = T_parent * T_anchor_parent * T_joint * (T_anchor_child)^-1
            
            # We traverse DFS from root
            transforms = {root_part: np.eye(4)}
            
            # Simplistic BFS/DFS traversal
            queue = [root_part]
            visited = {root_part}
            
            while queue:
                u = queue.pop(0)
                T_u_world = transforms[u]
                
                for v in G.successors(u):
                    if v in visited: continue
                    visited.add(v)
                    queue.append(v)
                    
                    j_data = G[u][v]["joint"]
                    # Is this the moving joint?
                    # Note: Need to check if this edge corresponds to target_joint_id
                    # We stored 'joint' dict, let's assume we can match by comparing dicts or ID if added
                    # Ideally store ID in edge properties
                    
                    current_angle = 0
                    if j_data == target_joint:
                        current_angle = angle
                        
                    # Calculate Edge Transform
                    anchor_p = self.anchors[u][j_data["parent_anchor"]]["matrix"]
                    anchor_c = self.anchors[v][j_data["child_anchor"]]["matrix"]
                    anchor_c_inv = np.linalg.inv(anchor_c)
                    
                    # Rotation (Z-axis of anchor frame)
                    R = trimesh.transformations.rotation_matrix(np.radians(current_angle), [0,0,1])
                    
                    # T_child = T_u * AnchorP * R * AnchorC_inv
                    T_v_world = T_u_world @ anchor_p @ R @ anchor_c_inv
                    transforms[v] = T_v_world

            # Check Collisions
            # Reset manager
            # Optimization: could use separate managers for static vs dynamic
            objs_to_check = []
            for part, mesh in meshes.items():
                if part in transforms:
                    tm = mesh.copy()
                    tm.apply_transform(transforms[part])
                    objs_to_check.append(tm)
            
            # Naive O(N^2) check via manager
            # Or assume only Target Child is moving against everything else?
            # If root is fixed, only descendants of target joint move.
            # Let's use the layout:
            # Fixed Group vs Moving Group
            
            # Identify moving parts (descendants of target joint)
            moving_subgraph = nx.descendants(G, target_joint["child_part"])
            moving_subgraph.add(target_joint["child_part"])
            
            moving_objs = []
            static_objs = []
            
            for part in all_parts:
                if part not in transforms: continue
                
                tm = meshes[part].copy()
                tm.apply_transform(transforms[part])
                
                if part in moving_subgraph:
                    moving_objs.append(tm)
                else:
                    static_objs.append(tm)
                    
            # Check
            cm_static = trimesh.collision.CollisionManager()
            for i, m in enumerate(static_objs):
                cm_static.add_object(f"static_{i}", m)
                
            for m in moving_objs:
                if cm_static.in_collision_internal(m):
                    collisions.append({"angle": float(angle), "collision": True})
                    break # Stop checking this angle, move to next
        
        return {
            "ok": True,
            "collisions_detected": len(collisions) > 0,
            "collision_count": len(collisions),
            "details": collisions
        }

    def _render_anchor_debug(self, origin, Z, X) -> str:
        fig = plt.figure(figsize=(4, 4))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter([origin[0]], [origin[1]], [origin[2]], c='k', s=50)
        
        # Z axis (Blue)
        scale = 10
        ax.quiver(origin[0], origin[1], origin[2], Z[0], Z[1], Z[2], length=scale, color='b', label='Z (Join/Hinge)')
        
        # X axis (Red)
        ax.quiver(origin[0], origin[1], origin[2], X[0], X[1], X[2], length=scale, color='r', label='X (Orient)')
        
        limit_range = 20
        ax.set_xlim(origin[0] - limit_range, origin[0] + limit_range)
        ax.set_ylim(origin[1] - limit_range, origin[1] + limit_range)
        ax.set_zlim(origin[2] - limit_range, origin[2] + limit_range)
        
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        plt.close(fig)
        buf.seek(0)
        return base64.b64encode(buf.read()).decode('utf-8')

    def fetch_artifact(self, session_id: str, artifact_name: str) -> Optional[Any]:
        # Same as before
        if session_id in self.session_cache and artifact_name in self.session_cache[session_id]:
            return self.session_cache[session_id][artifact_name]
        url = f"{self.api_url}/sessions/{session_id}/artifacts/{artifact_name}"
        if not url.endswith('.3mf'): url += '.3mf'
        try:
            resp = requests.get(url, timeout=10)
            if resp.status_code == 200:
                file_obj = io.BytesIO(resp.content)
                mesh = trimesh.load(file_obj, file_type='3mf')
                if session_id not in self.session_cache: self.session_cache[session_id] = {}
                self.session_cache[session_id][artifact_name] = mesh
                return mesh
        except: pass
        return None

    def insertion_path_check(self, session_id, part, assembly, direction, distance):
        # Stub for now
        return {"ok": True, "note": "Pending update for anchor logic"}
