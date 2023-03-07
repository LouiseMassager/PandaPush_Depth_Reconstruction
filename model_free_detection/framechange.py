import numpy as np


def translate(points,origin):
    for i in range(len(points)):
        points[i]=np.subtract(points[i],origin)
    return points

def get_rotation_matrix(i_v, unit=None):
    # From http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q38
    #https://stackoverflow.com/questions/43507491/imprecision-with-rotation-matrix-to-align-a-vector-to-an-axis
    #https://www.continuummechanics.org/rotationmatrix.html#:%7E:text=A%20transformation%20matrix%20describes%20the,the%20transpose%20of%20the%20other.
    if unit is None:
        unit = [0.0, 0.0, 1.0]
    # Normalize vector length
    i_v /= np.linalg.norm(i_v)

    # Get axis
    uvw = np.cross(i_v, unit)

    # compute trig values - no need to go through arccos and back
    rcos = np.dot(i_v, unit)
    rsin = np.linalg.norm(uvw)

    #normalize and unpack axis
    if not np.isclose(rsin, 0):
        uvw /= rsin
    u, v, w = uvw

    # Compute rotation matrix - re-expressed to show structure
    return (
        rcos * np.eye(3) +
        rsin * np.array([
            [ 0, -w,  v],
            [ w,  0, -u],
            [-v,  u,  0]
        ]) +
        (1.0 - rcos) * uvw[:,None] * uvw[None,:]
    )
def change_of_basis(points, direction):
    R=get_rotation_matrix(direction)
    #print(R)
    return np.transpose(np.dot(R,np.transpose(points)))
    
def align_to_origin(points,center,direction):
	points=translate(points,center)
	
	#print(np.mean(points,axis=0)) #should be [0 0 0]
	
	direction=direction/np.linalg.norm(direction) #normalize direction
	#print(np.linalg.norm(direction)) #should be 1
	return change_of_basis(points,direction)
	#change_of_basis(points,np.asarray([1,1,1])/np.linalg.norm(np.asarray([1,1,1])),direction)


#TEST:
#points=np.asarray([[1,2,4],[0,3,1]])
#center=np.mean(points,axis=0)
#print(align_to_origin(points,center,np.asarray([0,0,-1])))
	
