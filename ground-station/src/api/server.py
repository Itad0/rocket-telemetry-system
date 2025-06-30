"""
FastAPI Server for Ground Station API
"""

import logging
from typing import Dict, Any, Optional
from datetime import datetime
from fastapi import FastAPI, HTTPException, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel

logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Rocket Telemetry API",
    description="API for rocket telemetry ground station",
    version="1.0.0"
)

# Security
security = HTTPBearer()

# Request/Response models
class FaultInjectionRequest(BaseModel):
    fault_type: str
    duration: Optional[float] = None
    severity: Optional[float] = None
    
class TelemetryResponse(BaseModel):
    timestamp: str
    data: Dict[str, Any]
    
class SystemStatus(BaseModel):
    status: str
    uptime: float
    mode: str
    components: Dict[str, str]


# Global reference to ground station (set during app creation)
ground_station = None


def create_app(station):
    """Create FastAPI app with ground station reference"""
    global ground_station
    ground_station = station
    return app


def verify_token(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Verify API token"""
    # In production, verify against actual tokens
    if credentials.credentials != "demo-token":
        raise HTTPException(status_code=403, detail="Invalid authentication")
    return credentials.credentials


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Rocket Telemetry API",
        "version": "1.0.0",
        "status": "operational"
    }


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "service": "rocket-telemetry-api"
    }


@app.get("/api/v1/telemetry/current", response_model=TelemetryResponse)
async def get_current_telemetry(token: str = Depends(verify_token)):
    """Get current telemetry data"""
    if not ground_station:
        raise HTTPException(status_code=503, detail="Ground station not initialized")
        
    return TelemetryResponse(
        timestamp=datetime.now().isoformat(),
        data=ground_station.telemetry_data
    )


@app.get("/api/v1/system/status", response_model=SystemStatus)
async def get_system_status(token: str = Depends(verify_token)):
    """Get system status"""
    if not ground_station:
        raise HTTPException(status_code=503, detail="Ground station not initialized")
        
    components = {}
    for name, component in ground_station.components.items():
        if hasattr(component, 'get_stats'):
            stats = component.get_stats()
            components[name] = stats.get('status', 'unknown')
        else:
            components[name] = 'active'
            
    return SystemStatus(
        status="operational",
        uptime=ground_station.metrics.get('uptime', 0),
        mode=ground_station.mode.value,
        components=components
    )


@app.post("/api/v1/faults/inject")
async def inject_fault(
    request: FaultInjectionRequest,
    token: str = Depends(verify_token)
):
    """Inject a fault into the system"""
    if not ground_station:
        raise HTTPException(status_code=503, detail="Ground station not initialized")
        
    if 'fault' not in ground_station.components:
        raise HTTPException(status_code=501, detail="Fault injection not available")
        
    result = await ground_station.inject_fault(
        request.fault_type,
        {
            'duration': request.duration,
            'severity': request.severity
        }
    )
    
    if not result.get('success'):
        raise HTTPException(status_code=400, detail=result.get('message'))
        
    return result


@app.get("/api/v1/faults/active")
async def get_active_faults(token: str = Depends(verify_token)):
    """Get list of active faults"""
    if not ground_station:
        raise HTTPException(status_code=503, detail="Ground station not initialized")
        
    if 'fault' not in ground_station.components:
        return {"active_faults": []}
        
    return {
        "active_faults": ground_station.components['fault'].get_active_faults()
    }


@app.post("/api/v1/reports/generate")
async def generate_report(
    report_type: str = "flight",
    token: str = Depends(verify_token)
):
    """Generate a report"""
    if not ground_station:
        raise HTTPException(status_code=503, detail="Ground station not initialized")
        
    try:
        report_path = await ground_station.generate_report(report_type)
        return {
            "success": True,
            "report_path": report_path,
            "report_type": report_type
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/metrics")
async def get_metrics():
    """Get Prometheus metrics"""
    if not ground_station or 'prometheus' not in ground_station.components:
        return "# No metrics available\n"
        
    return ground_station.components['prometheus'].get_metrics()