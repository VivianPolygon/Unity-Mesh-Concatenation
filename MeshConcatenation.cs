//System Namespaces
using System;
using System.Collections.Generic;
using System.Numerics;
//Unity Namespaces
using UnityEngine;

namespace MeshConcatenation
{
    //aliases for carity of what is from unity
    using UnityV2 = UnityEngine.Vector2;
    using UnityV3 = UnityEngine.Vector3;
    using UnityV4 = UnityEngine.Vector4; 
    using UnitySubMeshDescriptor = UnityEngine.Rendering.SubMeshDescriptor;
    using UnityMaterial = Material;
    using UnityMesh = Mesh;
    using UnityMeshFilter = MeshFilter;
    using UnityMeshRenderer = MeshRenderer;

    /// <summary>
    /// Assign a mesh to these and add them together, and pull it back out to concatenate meshes
    /// </summary>
    public struct MeshDataSet
    {
        /// <summary>
        /// Get the material array on this meshDataSet
        /// </summary>
        public readonly UnityMaterial[] Materials => (UnityMaterial[])_matArr.Clone();

        public UnityV3[] _vertArr;
        private int[] _triArr;
        private UnityMaterial[] _matArr;
        public UnitySubMeshDescriptor[] _sMeshArr;
        private UnityV3[] _vertNormalArr;
        private UnityV4[] _vertTangentArr;
        private UnityV2[] _UVArr;
        private UnityV3 _AAABMinPoint;
        private UnityV3 _AAABMaxPoint;

        /// <summary>
        /// Generates the Unity Mesh from the MeshData
        /// </summary>
        /// <returns> returns the generated mesh </returns>
        public readonly UnityMesh GenerateMesh()
        {
            UnityMesh m = new()
            {
                vertices = _vertArr,
                triangles = _triArr,
                subMeshCount = _sMeshArr.Length,
                tangents = _vertTangentArr,
                normals = _vertNormalArr,
                uv = _UVArr,
                bounds = new((_AAABMinPoint + _AAABMaxPoint) * 0.5f, new(_AAABMaxPoint.x - _AAABMinPoint.x, _AAABMaxPoint.y - _AAABMinPoint.y, _AAABMaxPoint.z - _AAABMinPoint.z)),
            };
            m.SetSubMeshes(_sMeshArr);
            return m;
        }

        /// <summary>
        /// Create a mesh dataset from a prefab. Object must have a mesh filter and mesh renderer on the top-level object
        /// </summary>
        /// <param name="prefab"> prefab to create MeshDataSet from </param>
        /// <returns> the created MeshDataSet </returns>
        public static MeshDataSet CreateNewFromPrefab(GameObject prefab)
        {
            try
            {
                UnityMesh mesh = prefab.GetComponent<UnityMeshFilter>().sharedMesh;
                UnityMaterial[] mats = prefab.GetComponent<UnityMeshRenderer>().sharedMaterials;
                return new MeshDataSet(mesh, mats);
            }
            catch (Exception ex)
            {
                Debug.LogWarning("Unable to create a mesh from inputted GameObject. Object must have a mesh filter and mesh renderer on the top-level object");
                Debug.LogException(ex);
                return default;
            }
        }

        /// <summary>
        /// Create a new mesh data set from a mesh and it's materials
        /// </summary>
        /// <param name="mesh"> mesh to use </param>
        /// <param name="materials"> material(s) to use </param>
        public MeshDataSet(UnityMesh mesh, UnityMaterial[] materials)
        {
            //get the mesh data
            _vertArr = mesh.vertices;
            _triArr = mesh.triangles;
            //no function to get all submeshes, have to loop
            _sMeshArr = new UnitySubMeshDescriptor[mesh.subMeshCount];
            for (int i = 0; i < mesh.subMeshCount; i++) _sMeshArr[i] = mesh.GetSubMesh(i);
            //materials from mesh
            _matArr = materials;
            //surface vectors
            _vertTangentArr = mesh.tangents;
            _vertNormalArr = mesh.normals;
            //uvs
            _UVArr = mesh.uv;
            //Axis Aligned Bounding Box
            _AAABMinPoint = mesh.bounds.min;
            _AAABMaxPoint = mesh.bounds.max;
        }

        /// <summary>
        /// Combine two mesh data sets
        /// </summary>
        /// <param name="Mds1"> first mesh data set </param>
        /// <param name="Mds2"> second mesh data set </param>
        /// <returns> combined mesh data </returns>
        public static MeshDataSet operator + (MeshDataSet Mds1, MeshDataSet Mds2)
        {
            // Step 1 - Sort submeshes of both inputs by material into a dictionary

            int vertCount = 0; int triIndexCount = 0;
            //sorts submeshes by shared materials. Bool is used to determine which Mds it originally belonged to (true = Mds1, false = Mds2)
            Dictionary<UnityMaterial, List<(UnitySubMeshDescriptor, bool)>> subMeshMats = new();

            //gets all submeshes and materials from Mds1. also tracks vert and tri count
            for (int i = 0; i < Mds1._sMeshArr.Length; i++)
            {
                vertCount += Mds1._sMeshArr[i].vertexCount;
                triIndexCount += Mds1._sMeshArr[i].indexCount;
                if (subMeshMats.ContainsKey(Mds1._matArr[i]))
                {
                    subMeshMats[Mds1._matArr[i]].Add((Mds1._sMeshArr[i], true));
                }
                else
                {
                    List<(UnitySubMeshDescriptor, bool)> newSubmeshList = new() { (Mds1._sMeshArr[i], true) };
                    subMeshMats.Add(Mds1._matArr[i], newSubmeshList);
                }
            }

            //gets all submeshes and materials from Mds2. also tracks vert and tri count
            for (int i = 0; i < Mds2._sMeshArr.Length; i++)
            {
                vertCount += Mds2._sMeshArr[i].vertexCount;
                triIndexCount += Mds2._sMeshArr[i].indexCount;
                if (subMeshMats.ContainsKey(Mds2._matArr[i]))
                {
                    subMeshMats[Mds2._matArr[i]].Add((Mds2._sMeshArr[i], false));
                }
                else
                {
                    List<(UnitySubMeshDescriptor, bool)> newSubmeshList = new() { (Mds2._sMeshArr[i], false) };
                    subMeshMats.Add(Mds2._matArr[i], newSubmeshList);
                }
            }

            // Step 2 - Organize and combine data by mat, sequentially, key by key

            //Arrays that are set to the outputted mesh data
            UnityV3[] verticies = new UnityV3[vertCount];
            int[] triIndicies = new int[triIndexCount];
            UnityMaterial[] materials = new UnityMaterial[subMeshMats.Count];
            UnitySubMeshDescriptor[] submeshes = new UnitySubMeshDescriptor[materials.Length];
            UnityV3[] normals = new UnityV3[vertCount];
            UnityV4[] tangents = new UnityV4[vertCount];
            UnityV2[] uvs = new UnityV2[vertCount];

            //extra indexers used for keeping track of locations in different arrays 
            int matIndex = 0; int vertIndex = 0; int triIndex = 0;
            foreach (UnityMaterial mat in subMeshMats.Keys)
            {
                //the starting position in the index and triangle arrays is stored before creating submeshDescriptor for the material
                int vertexIndexStartPos = vertIndex;
                int triIndexStartPos = triIndex;
                //loops over material's list in dictionary
                for (int i = 0; i < subMeshMats[mat].Count; i++)
                {
                    UnitySubMeshDescriptor submesh = subMeshMats[mat][i].Item1;
                    //deterimes which mesh data set the submesh belongs to using stored boolean from earlier
                    MeshDataSet mds = subMeshMats[mat][i].Item2 ? Mds1 : Mds2;

                    //caculates the triangle index offset. Considering were moving verticies' positions in their array,
                    //we need to also offset the triangle indicies to still point to the correct verticies
                    int triOffset = vertIndex - submesh.firstVertex;
                    //loops over each vertex in the submesh, sets vertex data to their arrays
                    for (int j = submesh.firstVertex; j < submesh.firstVertex + submesh.vertexCount; j++)
                    {
                        verticies[vertIndex] = mds._vertArr[j];
                        tangents[vertIndex] = mds._vertTangentArr[j];
                        normals[vertIndex] = mds._vertNormalArr[j];
                        uvs[vertIndex] = mds._UVArr[j];

                        vertIndex++;
                    }
                    //loops over each triangle index, adding to shared array. Also applies triangle index offset.
                    for (int j = submesh.indexStart; j < submesh.indexStart + submesh.indexCount; j++)
                    {
                        triIndicies[triIndex] = mds._triArr[j] + triOffset;

                        triIndex++;
                    }
                }
                //creates the submesh descriptor for the current material
                UnitySubMeshDescriptor generatedSubmesh = new()
                {
                    firstVertex = vertexIndexStartPos,
                    vertexCount = vertIndex - vertexIndexStartPos,
                    indexStart = triIndexStartPos,
                    indexCount = triIndex - triIndexStartPos
                };

                //sets material and submesh index
                materials[matIndex] = mat;
                submeshes[matIndex] = generatedSubmesh;

                matIndex++;
            }

            // Step 3, Combine bounding boxes

            //combine bounding boxes
            UnityV3 min, max;

            min.x = Mds1._AAABMinPoint.x < Mds2._AAABMinPoint.x ? Mds1._AAABMinPoint.x : Mds2._AAABMinPoint.x;
            max.x = Mds1._AAABMaxPoint.x > Mds2._AAABMaxPoint.x ? Mds1._AAABMaxPoint.x : Mds2._AAABMaxPoint.x;
            min.y = Mds1._AAABMinPoint.y < Mds2._AAABMinPoint.y ? Mds1._AAABMinPoint.y : Mds2._AAABMinPoint.y;
            max.y = Mds1._AAABMaxPoint.y > Mds2._AAABMaxPoint.y ? Mds1._AAABMaxPoint.y : Mds2._AAABMaxPoint.y;
            min.z = Mds1._AAABMinPoint.z < Mds2._AAABMinPoint.z ? Mds1._AAABMinPoint.z : Mds2._AAABMinPoint.z;
            max.z = Mds1._AAABMaxPoint.z > Mds2._AAABMaxPoint.z ? Mds1._AAABMaxPoint.z : Mds2._AAABMaxPoint.z;

            // Step 4, Set to mesh data set and return
            MeshDataSet Mds3 = new()
            {
                _vertArr = verticies,
                _triArr = triIndicies,
                _matArr = materials,
                _sMeshArr = submeshes,

                _vertTangentArr = tangents,
                _vertNormalArr = normals,

                _UVArr = uvs,

                _AAABMinPoint = min,
                _AAABMaxPoint = max,
            };

            return Mds3;
        }

        /// <summary>
        /// Concatenate meshes. you can also use the + operator 
        /// </summary>
        /// <param name="Mds1"> first mesh data set </param>
        /// <param name="Mds2"> second mesh data set </param>
        /// <returns> combined mesh data </returns>
        public static MeshDataSet ConcatenateMeshes(MeshDataSet Mds1, MeshDataSet Mds2) => Mds1 + Mds2;

        /// <summary>
        /// Concatenate an arbitrary number of meshes. more efficient when concatenating in bulk
        /// </summary>
        /// <param name="MdsToConcatenate"> MeshDataSets to concatenate </param>
        /// <returns></returns>
        public static MeshDataSet ConcatenateMultipleMeshes (params MeshDataSet[] MdsToConcatenate)
        {
            if (MdsToConcatenate.Length == 0) { Debug.LogWarning("Unable to concatenate an array of Length 0"); return default; }
            else if (MdsToConcatenate.Length == 1) return MdsToConcatenate[0]; //no concatenation can occur if only 1 is inputted

            // Step 1 - Sort submeshes of both inputs by material into a dictionary. Also determine bounding box

            int vertCount = 0; int triIndexCount = 0;
            Dictionary<UnityMaterial, List<(UnitySubMeshDescriptor, int)>> subMeshMats = new();

            //combine bounding boxes
            UnityV3 min = MdsToConcatenate[0]._AAABMinPoint;
            UnityV3 max = MdsToConcatenate[0]._AAABMaxPoint;
            UnityV3 newMinCandidate, newMaxCandidate;

            for (int i = 0; i < MdsToConcatenate.Length; i++)
            {
                //combine the bounding boxes
                newMinCandidate = MdsToConcatenate[i]._AAABMinPoint;
                newMaxCandidate = MdsToConcatenate[i]._AAABMaxPoint;

                if (min.x > newMinCandidate.x) min.x = newMinCandidate.x;
                if (max.x < newMaxCandidate.x) max.x = newMaxCandidate.x;
                if (min.y > newMinCandidate.y) min.y = newMinCandidate.y;
                if (max.y < newMaxCandidate.y) max.y = newMaxCandidate.y;
                if (min.z > newMinCandidate.z) min.z = newMinCandidate.z;
                if (max.z < newMaxCandidate.z) max.z = newMaxCandidate.z;

                //sort submeshes
                for (int j = 0; j < MdsToConcatenate[i]._sMeshArr.Length; j++)
                {
                    vertCount += MdsToConcatenate[i]._sMeshArr[j].vertexCount;
                    triIndexCount += MdsToConcatenate[i]._sMeshArr[j].indexCount;
                    if (subMeshMats.ContainsKey(MdsToConcatenate[i]._matArr[j]))
                    {
                        subMeshMats[MdsToConcatenate[i]._matArr[j]].Add((MdsToConcatenate[i]._sMeshArr[j], i));
                    }
                    else
                    {
                        List<(UnitySubMeshDescriptor, int)> newSubmeshList = new() { (MdsToConcatenate[i]._sMeshArr[j], i) };
                        subMeshMats.Add(MdsToConcatenate[i]._matArr[j], newSubmeshList);
                    }
                }
            }

            // Step 2 - Organize and combine data by mat, sequentially, key by key

            //Arrays that are set to the outputted mesh data
            UnityV3[] verticies = new UnityV3[vertCount];
            int[] triIndicies = new int[triIndexCount];
            UnityMaterial[] materials = new UnityMaterial[subMeshMats.Count];
            UnitySubMeshDescriptor[] submeshes = new UnitySubMeshDescriptor[materials.Length];
            UnityV3[] normals = new UnityV3[vertCount];
            UnityV4[] tangents = new UnityV4[vertCount];
            UnityV2[] uvs = new UnityV2[vertCount];

            //extra indexers used for keeping track of locations in different arrays 
            int matIndex = 0; int vertIndex = 0; int triIndex = 0;
            foreach (UnityMaterial mat in subMeshMats.Keys)
            {
                //the starting position in the index and triangle arrays is stored before creating submeshDescriptor for the material
                int vertexIndexStartPos = vertIndex;
                int triIndexStartPos = triIndex;
                //loops over material's list in dictionary
                for (int i = 0; i < subMeshMats[mat].Count; i++)
                {
                    UnitySubMeshDescriptor submesh = subMeshMats[mat][i].Item1;
                    MeshDataSet mds = MdsToConcatenate[subMeshMats[mat][i].Item2];

                    //caculates the triangle index offset. Considering were moving verticies' positions in their array,
                    //we need to also offset the triangle indicies to still point to the correct verticies
                    int triOffset = vertIndex - submesh.firstVertex;
                    //loops over each vertex in the submesh, sets vertex data to their arrays
                    for (int j = submesh.firstVertex; j < submesh.firstVertex + submesh.vertexCount; j++)
                    {
                        verticies[vertIndex] = mds._vertArr[j];
                        tangents[vertIndex] = mds._vertTangentArr[j];
                        normals[vertIndex] = mds._vertNormalArr[j];
                        uvs[vertIndex] = mds._UVArr[j];

                        vertIndex++;
                    }
                    //loops over each triangle index, adding to shared array. Also applies triangle index offset.
                    for (int j = submesh.indexStart; j < submesh.indexStart + submesh.indexCount; j++)
                    {
                        triIndicies[triIndex] = mds._triArr[j] + triOffset;

                        triIndex++;
                    }
                }
                //creates the submesh descriptor for the current material
                UnitySubMeshDescriptor generatedSubmesh = new()
                {
                    firstVertex = vertexIndexStartPos,
                    vertexCount = vertIndex - vertexIndexStartPos,
                    indexStart = triIndexStartPos,
                    indexCount = triIndex - triIndexStartPos
                };

                //sets material and submesh index
                materials[matIndex] = mat;
                submeshes[matIndex] = generatedSubmesh;

                matIndex++;
            }

            // Step 3, Set to mesh data set and return
            MeshDataSet Mds3 = new()
            {
                _vertArr = verticies,
                _triArr = triIndicies,
                _matArr = materials,
                _sMeshArr = submeshes,
                _vertTangentArr = tangents,
                _vertNormalArr = normals,
                _UVArr = uvs,
                _AAABMinPoint = min,
                _AAABMaxPoint = max,
            };

            return Mds3;
        }


        /// <summary>
        /// Struct that transforms data in a mesh data set. applies scaling, rotation, and translation
        /// </summary>
        public struct VertexTransformation
        {
            /// <summary>
            /// Mesh data set to be transformed
            /// </summary>
            public MeshDataSet Mds;

            //Transform components
            /// <summary>
            /// Position offset from the orgin for the verticies - applied after scale and rotation. (X, Y, Z)
            /// </summary>
            public UnityV3 PositionOffset;
            /// <summary>
            /// Scale to apply to the object (X, Y, Z)
            /// </summary>
            public UnityV3 Scale;
            /// <summary>
            /// Degrees to rotate the verticeis around the AABB center (X, Y, Z)
            /// </summary>
            public UnityV3 RotationDeg;

            /// <summary>
            /// Create a new VertexTransformation for a mesh data set
            /// </summary>
            /// <param name="mds"> mesh data set for the transformation </param>
            public VertexTransformation(MeshDataSet mds)
            {
                // Standard Mds initialization
                Mds = mds;

                // Transform Initialization
                PositionOffset = UnityV3.zero;
                Scale = UnityV3.one;
                RotationDeg = UnityV3.zero;
            }

            /// <summary>
            /// Apply transformations to the contained mesh data.
            /// Does not modify original data assigned to the VertexTransformation object
            /// </summary>
            /// <returns> transformed data </returns>
            public readonly MeshDataSet GetTransformedMeshData()
            {
                // Step 1 - Update Bounding Box

                //create rotation quaterion
                RotationQuaternion rotation = new(new UnityV3(RotationDeg.x, RotationDeg.y, RotationDeg.z), true);
                rotation = rotation.Normalize();

                //converts scale vector to floats for increased speed in second vertex loop
                float scaleX = Scale.x;
                float scaleY = Scale.y;
                float scaleZ = Scale.z;

                //determins if the scale is uniform. If it is, some operations can be skipped
                bool scaleIsUniform = Mathf.Approximately(scaleX, scaleY) && Mathf.Approximately(scaleX, scaleZ);

                //rotate AABB min and max
                UnityV3 centerPoint = (Mds._AAABMinPoint + Mds._AAABMaxPoint) * 0.5f;
                float centerPointX = centerPoint.x;
                float centerPointY = centerPoint.y;
                float centerPointZ = centerPoint.z;

                //scale and rotate the minimum point
                UnityV3 newAABBMin = new((Mds._AAABMinPoint.x - centerPointX) * scaleX, (Mds._AAABMinPoint.y - centerPointY) * scaleY, (Mds._AAABMinPoint.z - centerPointZ) * scaleZ); //scales
                newAABBMin = RotationQuaternion.RotatePoint(newAABBMin, rotation); //rotates

                //scale and rotate the maximum point
                UnityV3 newAABBMax = new((Mds._AAABMaxPoint.x - centerPointX) * scaleX, (Mds._AAABMaxPoint.y - centerPointY) * scaleY, (Mds._AAABMaxPoint.z - centerPointZ) * scaleZ); //scales
                newAABBMax = RotationQuaternion.RotatePoint(newAABBMax, rotation); //rotates

                //ensure highest and lowest values are assigned appropiatly after rotations per-component
                if (newAABBMin.x > newAABBMax.x) (newAABBMin.x, newAABBMax.x) = (newAABBMax.x, newAABBMin.x);
                if (newAABBMin.y > newAABBMax.y) (newAABBMin.y, newAABBMax.y) = (newAABBMax.y, newAABBMin.y);
                if (newAABBMin.z > newAABBMax.z) (newAABBMin.z, newAABBMax.z) = (newAABBMax.z, newAABBMin.z);

                //Step 2 - Apply Transformations

                //output array initialization
                UnityV3[] transformedVertices = new UnityV3[Mds._vertArr.Length];
                UnityV3[] transformedNormals = new UnityV3[Mds._vertArr.Length];
                UnityV4[] transformedTangents = new UnityV4[Mds._vertArr.Length];

                //prevents 0 scale rendering errors, and dividing by 0 when adjusting surface normals
                float surfaceNormalScaleX = scaleX == 0 ? float.MaxValue : (1f / scaleX);
                float surfaceNormalScaleY = scaleY == 0 ? float.MaxValue : (1f / scaleY);
                float surfaceNormalScaleZ = scaleZ == 0 ? float.MaxValue : (1f / scaleZ);

                //loops over each vertex, applying transformations
                for (int i = 0; i < Mds._vertArr.Length; i++)
                {
                    UnityV3 vert = Mds._vertArr[i]; //gets vertex
                    UnityV3 normal = Mds._vertNormalArr[i]; //gets normal
                    UnityV3 tangent = new (Mds._vertTangentArr[i].x, Mds._vertTangentArr[i].y, Mds._vertTangentArr[i].z); //gets tangent, excludes w component used for bitangent

                    // Apply scale to vertex
                    vert = new UnityV3((vert.x - centerPointX) * scaleX, (vert.y - centerPointY) * scaleY, (vert.z - centerPointZ) * scaleZ);
                    //scale to normal and tangent
                    if (!scaleIsUniform) //if the scale is non uniform, adjustment to normals and tangents are needed
                    {
                        normal = new UnityV3(normal.x * surfaceNormalScaleX, normal.y * surfaceNormalScaleY, normal.z * surfaceNormalScaleZ).normalized;  //adjusts normal for nonuniform scaling
                        tangent = new UnityV3(tangent.x * scaleX, tangent.y * scaleY, tangent.z * scaleZ).normalized; //adjusts tangent for nonuniform scaling
                    }

                    // Apply Rotation to Vertex
                    //rotates vertice position
                    vert = RotationQuaternion.RotatePoint(vert, rotation);
                    //rotates normal and tangent
                    normal = RotationQuaternion.RotatePoint(normal, rotation); //rotates normal based on mesh rotation
                    tangent = RotationQuaternion.RotatePoint(tangent, rotation); //rotates normal based on mesh rotation

                    // Apply Translation
                    vert += PositionOffset;

                    // Set to output
                    transformedVertices[i] = vert; //sets to output
                    transformedNormals[i] = normal; //normRotation; //sets to output
                    transformedTangents[i] = new UnityV4(tangent.x, tangent.y, tangent.z, Mds._vertTangentArr[i].w);
                }

                //return modified data
                MeshDataSet MdsR = Mds;
                MdsR._vertArr = transformedVertices;
                MdsR._vertNormalArr = transformedNormals;
                MdsR._vertTangentArr = transformedTangents;

                MdsR._AAABMinPoint = newAABBMin;
                MdsR._AAABMaxPoint = newAABBMax;

                return MdsR;
            }

            /// <summary>
            /// Partial implementation of a quaternion capable of necessary operations for rotation.
            /// Uses Unity's conventions
            /// - Left handed coordinate system
            /// - ZXY rotation order
            /// - Extrinsic rotations
            /// </summary>
            public struct RotationQuaternion
            {
                //conversion factor for euler angles in degrees to half angles of radians
                const float EULER_TO_HALFANGLES = Mathf.Deg2Rad * 0.5f;
                //static point rotation quaternion removes need for redundant assignment to q0 when rotating a point.
                //provides performance increase when rotating many points
                private static RotationQuaternion _pointRotation = new() { q0 = 0 };

                /// <summary>
                /// real component of the quaternion
                /// </summary>
                public double q0;

                /// <summary>
                /// Imaginary component of the quaternion
                /// </summary>
                public Complex q1i, q2j, q3k;

                /// <summary>
                /// Create a rotation quaternion from a set of euler angles measured in degrees or radians
                /// </summary>
                /// <param name="angles"> angles to convert (X,Y,Z) </param>
                /// <param name="degrees"> if true, treats angles as degrees, if false, treats angles as radians. 
                /// Converts to radians inside the function if true </param>
                public RotationQuaternion(UnityV3 angles, bool degrees = true)
                {
                    //converts to radians if inputted as degrees, and converts to half angles
                    if (degrees) angles *= EULER_TO_HALFANGLES;
                    else angles *= 0.5f;

                    //per axis quaternions
                    RotationQuaternion X = new(new UnityV4(Mathf.Sin(angles.x), 0, 0, Mathf.Cos(angles.x)));
                    RotationQuaternion Y = new(new UnityV4(0, Mathf.Sin(angles.y), 0, Mathf.Cos(angles.y)));
                    RotationQuaternion Z = new(new UnityV4(0, 0, Mathf.Sin(angles.z), Mathf.Cos(angles.z)));

                    //rotated by ZXY convention (unity uses this)
                    //multiplication happens in inverse order of convention
                    this = Y * X * Z;
                }



                /// <summary>
                /// Create a rotation quaternion from a 4 component vector.
                /// W is set to the real component, while X, Y, and Z are set to i, j, and k respectively
                /// </summary>
                /// <param name="components"> 4 component vector (X, Y, Z, W) </param>
                public RotationQuaternion(UnityV4 components)
                {
                    q0 = components.w;
                    q1i = new Complex(0, components.x);
                    q2j = new Complex(0, components.y);
                    q3k = new Complex(0, components.z);
                }

                /// <summary>
                /// Normalize a rotation quaternion
                /// </summary>
                /// <returns> The normalized quaternion </returns>
                public RotationQuaternion Normalize()
                {
                    // a normalized/unit quaternion definition: q0^2 + q1i^2 + q2j^2 + q3k^2 = 1
                    float mag = Mathf.Sqrt((float)((q0 * q0) + (q1i.Imaginary * q1i.Imaginary) + (q2j.Imaginary * q2j.Imaginary) + (q3k.Imaginary * q3k.Imaginary)));
                    //quaternion is already a unit quaternion
                    if (Mathf.Approximately(mag, 1.0f)) return this;
                    //imposible to normalize four zeros, return identity
                    else if (Mathf.Approximately(mag, 0)) return Identity();
                    //normalize by dividing by that magnitude, proportionaly scaling components into a unit quaternion
                    else
                    {
                        q0 /= mag;
                        q1i /= mag;
                        q2j /= mag;
                        q3k /= mag;
                    }
                    return this;
                }

                /// <summary>
                /// Rotates a point in 3D space by the quaternion. point is rotated around the orgin.
                /// ex: (2, 3, 1) will be rotated around (0,0,0)
                /// </summary>
                /// <param name="point"> point in 3D space </param>
                /// <param name="rotation"> rotation quaternion to apply to the point </param>
                /// <returns> new rotated point </returns>
                public static UnityV3 RotatePoint(UnityV3 point, RotationQuaternion rotation)
                {
                    _pointRotation.q1i = new(0, point.x);
                    _pointRotation.q2j = new(0, point.y);
                    _pointRotation.q3k = new(0, point.z);

                    //sandwhich product used for extrinsic rotation
                    RotationQuaternion roQ = rotation * _pointRotation * Invert(rotation);
                    return new UnityV3((float)roQ.q1i.Imaginary, (float)roQ.q2j.Imaginary, (float)roQ.q3k.Imaginary);
                }

                /// <summary>
                /// Inverts a rotation quaternion. (negates the imaginary components)
                /// </summary>
                /// <param name="rq"> rotation quaternion to invert </param>
                /// <returns> inverted quaternion </returns>
                public static RotationQuaternion Invert(RotationQuaternion rq)
                {
                    //negates imaginary components
                    rq.q1i = -rq.q1i;
                    rq.q2j = -rq.q2j;
                    rq.q3k = -rq.q3k;
                    return rq;
                }

                /// <summary>
                /// Multiply two quaternions, effectively "adds their rotations"
                /// Note that quaternion multiplication is non-commutative -
                /// a * b != b * a
                /// </summary>
                /// <param name="a"> first quaternion </param>
                /// <param name="b"> second quaternion </param>
                /// <returns> product of a * b </returns>
                public static RotationQuaternion operator *(RotationQuaternion a, RotationQuaternion b)
                {
                    //seperates imaginary component into seperate vectors
                    DoubleV3 rVec = new(a.q1i.Imaginary, a.q2j.Imaginary, a.q3k.Imaginary);
                    DoubleV3 sVec = new(b.q1i.Imaginary, b.q2j.Imaginary, b.q3k.Imaginary);

                    //multiplies the vector components
                    DoubleV3 vector = b.q0 * rVec + a.q0 * sVec + DoubleV3.DoubleCross(rVec, sVec);

                    //sets to and returns b as the product
                    b.q0 = b.q0 * a.q0 - DoubleV3.DoubleDot(sVec, rVec);
                    b.q1i = new(0, vector.x);
                    b.q2j = new(0, vector.y);
                    b.q3k = new(0, vector.z);
                    return b;
                }

                /// <summary>
                /// identity of the quaternion (x = 0, y = 0, z = 0, w = 1)
                /// </summary>
                /// <returns> identity quaternion </returns>
                public static RotationQuaternion Identity()
                {
                    return new RotationQuaternion
                    {
                        q0 = 1.0f,
                        q1i = Complex.Zero,
                        q2j = Complex.Zero,
                        q3k = Complex.Zero,
                    };
                }

                /// <summary>
                /// Implementation of a 3D vector using the double type.
                /// Reduces casting between floats and doubles when working with complex numbers
                /// </summary>
                private struct DoubleV3
                {
                    /// <summary>
                    /// Vector component
                    /// </summary>
                    public double x, y, z;

                    /// <summary>
                    /// Create a new 3D vector with doubles
                    /// </summary>
                    /// <param name="x"> X component </param>
                    /// <param name="y"> Y component </param>
                    /// <param name="z"> Z component </param>
                    public DoubleV3(double x, double y, double z)
                    {
                        this.x = x; this.y = y; this.z = z;
                    }

                    /// <summary>
                    /// Get the cross product of two double V3s
                    /// </summary>
                    /// <param name="a"> First vector </param>
                    /// <param name="b"> Second vector </param>
                    /// <returns> a X b </returns>
                    public static DoubleV3 DoubleCross(DoubleV3 a, DoubleV3 b)
                    {
                        return new()
                        {
                            x = a.y * b.z - a.z * b.y,
                            y = -(a.x * b.z - a.z * b.x),
                            z = a.x * b.y - a.y * b.x
                        };
                    }

                    /// <summary>
                    /// Get the dot product of two double V3s
                    /// </summary>
                    /// <param name="a"> First vector </param>
                    /// <param name="b"> Second vector </param>
                    /// <returns> a . b </returns>
                    public static double DoubleDot(DoubleV3 a, DoubleV3 b)
                    {
                        return a.x * b.x + a.y * b.y + a.z * b.z;
                    }

                    /// <summary>
                    /// Multiply a double V3 by a scalar
                    /// </summary>
                    /// <param name="scalar"> scaler to multiply the V3 by </param>
                    /// <param name="V"> Vector to scale </param>
                    /// <returns> scalar * V </returns>
                    public static DoubleV3 operator *(double scalar, DoubleV3 V)
                    {
                        V.x *= scalar;
                        V.y *= scalar;
                        V.z *= scalar;
                        return V;
                    }

                    /// <summary>
                    /// Add two double V3s together component-wise 
                    /// </summary>
                    /// <param name="a"> First vector </param>
                    /// <param name="b"> Second vector </param>
                    /// <returns> a + b </returns>
                    public static DoubleV3 operator +(DoubleV3 a, DoubleV3 b)
                    {
                        b.x += a.x;
                        b.y += a.y;
                        b.z += a.z;
                        return b;
                    }
                }
            }
        }
    }

}
