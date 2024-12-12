from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPipelineResult import PhotonPipelineResult, PhotonTrackedTarget
from constants import visionConstants as c

class VisionSubsystem:
    
    def __init__(self):
        self.limelight = PhotonCamera(c.cameraName)
        
    def getPhotonResults(self) -> PhotonPipelineResult:
        return self.limelight.getLatestResult()
    
    def getBestResult(self) -> PhotonTrackedTarget:
        photonResult: list[PhotonTrackedTarget] = self.getPhotonResults().getTargets()
        
        bestResult: PhotonTrackedTarget = None
        tracker = 0
        for result in photonResult:
            if result.getArea() > tracker:
                bestResult = result
                tracker = bestResult.getArea()
        return bestResult