# 36 "/usr/local/include/OGRE/OgreEdgeListBuilder.h" 2

namespace Ogre {


    /** This class contains the information required to describe the edge connectivity of a
        given set of vertices and indexes. 
    @remarks 
        This information is built using the EdgeListBuilder class. Note that for a given mesh,
        which can be made up of multiple submeshes, there are separate edge lists for when 
    */
 class EdgeData : public EdgeDataAlloc
    {
    public:
        /** Basic triangle structure. */
        struct Triangle {
            /** The set of indexes this triangle came from (NB it is possible that the triangles on 
               one side of an edge are using a different vertex buffer from those on the other side.) */
            size_t indexSet;
            /** The vertex set these vertices came from. */
            size_t vertexSet;
            size_t vertIndex[3];/// Vertex indexes, relative to the original buffer
            size_t sharedVertIndex[3]; /// Vertex indexes, relative to a shared vertex buffer with 
                                        // duplicates eliminated (this buffer is not exposed)
        };
        /** Edge data. */
        struct Edge {
            /** The indexes of the 2 tris attached, note that tri 0 is the one where the 
                indexes run _anti_ clockwise along the edge. Indexes must be
                reversed for tri 1. */
            size_t triIndex[2];
            /** The vertex indices for this edge. Note that both vertices will be in the vertex
                set as specified in 'vertexSet', which will also be the same as tri 0 */
            size_t vertIndex[2];
            /** Vertex indices as used in the shared vertex list, not exposed. */
            size_t sharedVertIndex[2];
            /** Indicates if this is a degenerate edge, ie it does not have 2 triangles */
            bool degenerate;
        };

        // Array of 4D vector of triangle face normal, which is unit vector orthogonal
        // to the triangles, plus distance from origin.
        // Use aligned policy here because we are intended to use in SIMD optimised routines .
        typedef std::vector<Vector4, STLAllocator<Vector4, CategorisedAlignAllocPolicy<MEMCATEGORY_GEOMETRY> > > TriangleFaceNormalList;

        // Working vector used when calculating the silhouette.
        // Use std::vector<char> instead of std::vector<bool> which might implemented
        // similar bit-fields causing loss performance.
        typedef std::vector<char> TriangleLightFacingList;

        typedef std::vector<Triangle> TriangleList;
        typedef std::vector<Edge> EdgeList;

        /** A group of edges sharing the same vertex data. */
        struct EdgeGroup
        {
            /** The vertex set index that contains the vertices for this edge group. */
            size_t vertexSet;
            /** Pointer to vertex data used by this edge group. */
            __const__ VertexData* vertexData;
            /** Index to main triangles array, indicate the first triangle of this edge
                group, and all triangles of this edge group are stored continuous in
                main triangles array.
            */
            size_t triStart;
            /** Number triangles of this edge group. */
            size_t triCount;
            /** The edges themselves. */
            EdgeList edges;

        };

        typedef std::vector<EdgeGroup> EdgeGroupList;

        /** Main triangles array, stores all triangles of this edge list. Note that
            triangles are grouping against edge group.
        */
        TriangleList triangles;
        /** All triangle face normals. It should be 1:1 with triangles. */
        TriangleFaceNormalList triangleFaceNormals;
        /** Triangle light facing states. It should be 1:1 with triangles. */
        TriangleLightFacingList triangleLightFacings;
        /** All edge groups of this edge list. */
        EdgeGroupList edgeGroups;
        /** Flag indicate the mesh is manifold. */
        bool isClosed;


        /** Calculate the light facing state of the triangles in this edge list
        @remarks
            This is normally the first stage of calculating a silhouette, i.e.
            establishing which tris are facing the light and which are facing
            away. This state is stored in the 'triangleLightFacings'.
        @param lightPos 4D position of the light in object space, note that 
            for directional lights (which have no position), the w component
            is 0 and the x/y/z position are the direction.
        */
        void updateTriangleLightFacing(__const__ Vector4& lightPos);
        /** Updates the face normals for this edge list based on (changed)
            position information, useful for animated objects. 
        @param vertexSet The vertex set we are updating
        @param positionBuffer The updated position buffer, must contain ONLY xyz
        */
        void updateFaceNormals(size_t vertexSet, __const__ HardwareVertexBufferSharedPtr& positionBuffer);



        // Debugging method
        void log(Log* log);

    };

    /** General utility class for building edge lists for geometry.
    @remarks
        You can add multiple sets of vertex and index data to build and edge list. 
        Edges will be built between the various sets as well as within sets; this allows 
        you to use a model which is built from multiple SubMeshes each using 
        separate index and (optionally) vertex data and still get the same connectivity 
        information. It's important to note that the indexes for the edge will be constrained
        to a single vertex buffer though (this is required in order to render the edge).
    */
    class EdgeListBuilder
    {
    public:

        EdgeListBuilder();
        virtual ~EdgeListBuilder();
        /** Add a set of vertex geometry data to the edge builder. 
        @remarks
            You must add at least one set of vertex data to the builder before invoking the
            build method.
        */
        void addVertexData(__const__ VertexData* vertexData);
        /** Add a set of index geometry data to the edge builder. 
        @remarks
            You must add at least one set of index data to the builder before invoking the
            build method.
        @param indexData The index information which describes the triangles.
        @param vertexSet The vertex data set this index data refers to; you only need to alter this
            if you have added multiple sets of vertices
        @param opType The operation type used to render these indexes. Only triangle types
            are supported (no point or line types)
        */
        void addIndexData(__const__ IndexData* indexData, size_t vertexSet = 0,
            RenderOperation::OperationType opType = RenderOperation::OT_TRIANGLE_LIST);

        /** Builds the edge information based on the information built up so far.
        @remarks
            The caller takes responsibility for deleting the returned structure.
        */
        EdgeData* build(void);

        /// Debugging method
        void log(Log* l);
    protected:

        /** A vertex can actually represent several vertices in the final model, because
		vertices along texture seams etc will have been duplicated. In order to properly
		evaluate the surface properties, a single common vertex is used for these duplicates,
		and the faces hold the detail of the duplicated vertices.
		*/
        struct CommonVertex {
            Vector3 position; // location of point in euclidean space
         size_t index; // place of vertex in common vertex list
            size_t vertexSet; // The vertex set this came from
            size_t indexSet; // The index set this was referenced (first) from
            size_t originalIndex; // place of vertex in original vertex set
        };
        /** A set of indexed geometry data */
        struct Geometry {
            size_t vertexSet; // The vertex data set this geometry data refers to
            size_t indexSet; // The index data set this geometry data refers to
            __const__ IndexData* indexData; // The index information which describes the triangles.
            RenderOperation::OperationType opType; // The operation type used to render this geometry
        };
        /** Comparator for sorting geometries by vertex set */
        struct geometryLess {
            bool operator()(__const__ Geometry& a, __const__ Geometry& b) __const__
            {
                if (a.vertexSet < b.vertexSet) return true;
                if (a.vertexSet > b.vertexSet) return false;
                return a.indexSet < b.indexSet;
            }
        };
        /** Comparator for unique vertex list */
        struct vectorLess {
            bool operator()(__const__ Vector3& a, __const__ Vector3& b) __const__
            {
                if (a.x < b.x) return true;
                if (a.x > b.x) return false;
                if (a.y < b.y) return true;
                if (a.y > b.y) return false;
                return a.z < b.z;
            }
        };

        typedef std::vector<__const__ VertexData*> VertexDataList;
        typedef std::vector<Geometry> GeometryList;
        typedef std::vector<CommonVertex> CommonVertexList;

        GeometryList mGeometryList;
        VertexDataList mVertexDataList;
        CommonVertexList mVertices;
        EdgeData* mEdgeData;
  /// Map for identifying common vertices
  typedef std::map<Vector3, size_t, vectorLess> CommonVertexMap;
  CommonVertexMap mCommonVertexMap;
        /** Edge map, used to connect edges. Note we allow many triangles on an edge,
        after connected an existing edge, we will remove it and never used again.
        */
        typedef std::multimap< std::pair<size_t, size_t>, std::pair<size_t, size_t> > EdgeMap;
        EdgeMap mEdgeMap;

        void buildTrianglesEdges(__const__ Geometry &geometry);

        /// Finds an existing common vertex, or inserts a new one
        size_t findOrCreateCommonVertex(__const__ Vector3& vec, size_t vertexSet,
            size_t indexSet, size_t originalIndex);
        /// Connect existing edge or create a new edge - utility method during building
        void connectOrCreateEdge(size_t vertexSet, size_t triangleIndex, size_t vertIndex0, size_t vertIndex1,
            size_t sharedVertIndex0, size_t sharedVertIndex1);
    };

}

