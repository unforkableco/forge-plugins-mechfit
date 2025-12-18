from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import uvicorn
from src.mech_engine import MechEngine

app = FastAPI()

# Configuration
FABRIKATOR_API_URL = os.environ.get("FABRIKATOR_API_URL", "http://host.docker.internal:3000")

# Initialize Engine
engine = MechEngine(FABRIKATOR_API_URL)

# Models
class Context(BaseModel):
    sessionId: str
    projectId: Optional[str] = None
    
class DefineAnchorParams(BaseModel):
    part: str
    anchor_id: str
    origin: List[float]
    z_axis: List[float]
    x_axis: List[float]

class DefineAnchorRequest(BaseModel):
    context: Context
    args: DefineAnchorParams

class DefineJointParams(BaseModel):
    joint_id: str
    type: str # revolute, prismatic, fixed
    parent_part: str
    parent_anchor: str
    child_part: str
    child_anchor: str
    limits: List[float] # [min, max]

class DefineJointRequest(BaseModel):
    context: Context
    args: DefineJointParams
    
class MotionSweepParams(BaseModel):
    assembly: str
    joints: List[str]
    step_deg: Optional[float] = 5.0
    step_mm: Optional[float] = 1.0
    report_contacts: Optional[bool] = False

class MotionSweepRequest(BaseModel):
    context: Context
    args: MotionSweepParams
    
class InsertionPathParams(BaseModel):
    part: str
    assembly: str
    direction: List[float]
    distance: float

class InsertionPathRequest(BaseModel):
    context: Context
    args: InsertionPathParams

# Endpoints

@app.get("/health")
def health_check():
    return {"status": "ok", "service": "mechfit"}

@app.post("/tools/define_anchor")
async def define_anchor(req: DefineAnchorRequest):
    try:
        result = engine.define_anchor(
            req.args.part,
            req.args.anchor_id,
            req.args.origin,
            req.args.z_axis,
            req.args.x_axis
        )
        return {"result": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/tools/define_joint")
async def define_joint(req: DefineJointRequest):
    try:
        result = engine.define_joint(
            req.args.joint_id,
            req.args.type,
            req.args.parent_part,
            req.args.parent_anchor,
            req.args.child_part,
            req.args.child_anchor,
            req.args.limits
        )
        return {"result": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/tools/motion_sweep_check")
async def motion_sweep_check(req: MotionSweepRequest):
    try:
        result = engine.motion_sweep_check(
            req.context.sessionId,
            req.args.joints,
            req.args.step_deg
        )
        return {
            "result": result
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/tools/insertion_path_check")
async def insertion_path_check(req: InsertionPathRequest):
    try:
        result = engine.insertion_path_check(
            req.context.sessionId,
            req.args.part,
            req.args.assembly,
            req.args.direction,
            req.args.distance
        )
        return {
            "result": result
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
