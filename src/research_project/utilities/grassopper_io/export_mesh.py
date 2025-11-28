import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
from compas.datastructures import Mesh


# https://github.com/compas-dev/compas/issues/512


class MeshExport:
    def __init__(self, GH_Mesh, Folder_Path, file_format):
        for i, mesh in enumerate(GH_Mesh):
            if not isinstance(mesh, rg.Mesh):
                raise ValueError("Input must be a mesh")
            if file_format == "OBJ":
                Path = Folder_Path + str(i) + ".obj"
                self.mesh_to_OBJ(mesh, Path)
            elif file_format == "STL":
                Path = Folder_Path + str(i) + ".stl"
                self.mesh_to_STL(mesh, Path)

    def rMesh2cMesh(self, rg_mesh):
        if isinstance(rg_mesh, rg.Mesh):
            # single mesh
            compas_mesh_vertices = [(v.X, v.Y, v.Z) for v in rg_mesh.Vertices]
            compas_mesh_faces = rg_mesh.Faces.ToIntArray(True)
            compas_mesh_faces_sorted = []
            for i in range(0, len(compas_mesh_faces), 3):
                face = [compas_mesh_faces[i], compas_mesh_faces[i + 1], compas_mesh_faces[i + 2]]
                compas_mesh_faces_sorted.append(face)
            return Mesh.from_vertices_and_faces(compas_mesh_vertices, compas_mesh_faces_sorted)
        else:
            # array of meshes
            meshes = []
            for m in rg_mesh:
                compas_mesh_vertices = [(v.X, v.Y, v.Z) for v in m.Vertices]
                compas_mesh_faces = m.Faces.ToIntArray(True)
                compas_mesh_faces = [compas_mesh_faces[i:i + 3] for i in range(0, len(compas_mesh_faces), 3)]
                meshes.append(Mesh.from_vertices_and_faces(compas_mesh_vertices, compas_mesh_faces))
            return meshes

    def mesh_to_OBJ(self, mesh_to_export, File_Path):
        # Create a FileWriteOptions object
        Compas_Mesh = self.rMesh2cMesh(mesh_to_export)
        Compas_Mesh.to_obj(File_Path)

    def mesh_to_STL(self, mesh_to_export, File_Path):
        # Write an ASCII format STL file for the input mesh.
        if not isinstance(mesh_to_export, rg.Mesh):
            try:
                mesh_to_export = rs.coercegeometry(mesh_to_export)
            except Exception:
                raise ValueError("Input must be a mesh")
        mesh_to_export.Normals.ComputeNormals()  # NITM ("Not In The Manual") but this is needed first.
        verts = mesh_to_export.Vertices  # Get all vertices of the mesh from Rhino.

        stl_file = open("%s" % File_Path, "w", encoding="utf-8")
        stl_file.write("solid OBJECT\n")
        for i, face in enumerate(mesh_to_export.Faces):  # Rhino gives faces by vertex index number.
            stl_file.write("  facet normal %s\n" % str(mesh_to_export.FaceNormals[i]).replace(",", " "))  # Rhino gives normals!
            stl_file.write("    outer loop\n")
            stl_file.write("      vertex %s\n" % str(verts[face.A]).replace(",", " "))  # Rhino has ABCD properties for face vertex index numbers.
            stl_file.write("      vertex %s\n" % str(verts[face.B]).replace(",", " "))
            stl_file.write("      vertex %s\n" % str(verts[face.C]).replace(",", " "))
            stl_file.write("    endloop\n")
            stl_file.write("  endfacet\n")
            if face.IsQuad:
                stl_file.write("  facet normal %s\n" % str(mesh_to_export.FaceNormals[i]).replace(",", " "))
                stl_file.write("    outer loop\n")
                stl_file.write("      vertex %s\n" % str(verts[face.C]).replace(",", " "))
                stl_file.write("      vertex %s\n" % str(verts[face.D]).replace(",", " "))
                stl_file.write("      vertex %s\n" % str(verts[face.A]).replace(",", " "))
                stl_file.write("    endloop\n")
                stl_file.write("  endfacet\n")
        stl_file.write("endsolid OBJECT\n")
        stl_file.close()
