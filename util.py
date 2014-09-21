import vtk

# This function combines a rotation matrix
# and a position vector, as obtained by ODE
# into a vtkMatrix4x4 as required by VTK
def rotpos_to_vtkMatrix4x4(r,p):
    m = vtk.vtkMatrix4x4()
    m.DeepCopy((r[0], r[1], r[2], p[0],\
                r[3], r[4], r[5], p[1],\
                r[6], r[7], r[8], p[2],\
                0.0,  0.0,  0.0, 1.0))
    return m

