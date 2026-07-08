/*
© Siemens AG, 2017-18
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using System.Collections.Generic;
using System.IO;

namespace UrdfToolkit.Urdf.Importer
{
    public static class StlImporter
    {
        public static Mesh[] ImportMesh(string path)
        {
            IList<StlReader.Facet> facets;
            if (IsBinary(path))
                facets = StlReader.ReadBinaryFile(path);
            else
                facets = StlReader.ReadAsciiFile(path);

            return CreateMesh(facets);
        }

        private static bool IsBinary(string path)
        {
            int maxCharsToCheck = 100;

            using (StreamReader reader = new StreamReader(path))
                for (int i = 0; i < maxCharsToCheck; i++)
                    if (reader.Read() == '\0')
                        return true;

            return false;
        }

        private static Mesh[] CreateMesh(IList<StlReader.Facet> facets)
        {
            int totalNumberOfFacets = facets.Count;
            int vertexCount = totalNumberOfFacets * 3;
            int[] order = new int[] { 0, 2, 1 };

            Vector3[] vertices = new Vector3[vertexCount];
            Vector3[] normals = new Vector3[vertexCount];
            int[] triangles = new int[vertexCount];

            for (int facetIndex = 0; facetIndex < totalNumberOfFacets; facetIndex++)
            {
                for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
                {
                    int index = facetIndex * 3 + vertexIndex;
                    vertices[index] = facets[facetIndex].vertices[order[vertexIndex]];
                    normals[index] = facets[facetIndex].normal;
                    triangles[index] = index;
                }
            }

            Mesh mesh = new Mesh();
            // UInt16 caps a mesh at 65535 vertices, UInt32 lets large STL files import as a single mesh
            mesh.indexFormat = vertexCount > 65535 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16;
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.triangles = triangles;

            return new Mesh[] { mesh };
        }
    }
}