// Copyright Michael Chapman 2018
#pragma once

#include "CoreMinimal.h"
#include "Navmesh/RecastNavMesh.h"
#include "Detour/DetourNavMeshBuilder.h"
#include "RegionDistribution.h"
#include "ManualDetourNavMesh.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogManualNavMesh, Log, All);

class dtNavMesh;
struct dtNavMeshParams;
class AWorldMap;
/*
// Need to mark which points are on the border
// 10000 = no border,
static const uint8 DNM_NoBorder = 0x80;
// 01000 = north,
static const uint8 DNM_North = 0x40;
// 00100 = east,
static const uint8 DNM_East = 0x20;
// 00010 = south,
static const uint8 DNM_South = 0x10;
// 00001 = west
static const uint8 DNM_West = 0x08;
*/

UENUM(BlueprintType)
enum class EPCGDirection : uint8
{
	E_North UMETA(DisplayName = "North"),
	E_South UMETA(DisplayName = "South"),
	E_East  UMETA(DisplayName = "East"),
	E_West  UMETA(DisplayName = "West"),
	E_None  UMETA(DisplayName = "None"),
};

UENUM(BlueprintType)
enum class EPCGTriPoint : uint8
{
	E_Internal  UMETA(DisplayName = "Internal"),
	E_External  UMETA(DisplayName = "External"),
	E_NorthWest UMETA(DisplayName = "NorthWest"),
	E_North		UMETA(DisplayName = "North"),
	E_NorthEast UMETA(DisplayName = "NorthEast"),
	E_East		UMETA(DisplayName = "East"),
	E_SouthEast UMETA(DisplayName = "SouthEast"),
	E_South		UMETA(DisplayName = "South"),
	E_SouthWest UMETA(DisplayName = "SouthWest"),
	E_West		UMETA(DisplayName = "West")
};

// Each part of the triangle that can result in a point
// - each point
// - each direction of each edge 
// - each corner of the bounds
UENUM(BlueprintType)
enum class EPCGTriMember : uint8
{
	E_A  UMETA(DisplayName = "A"),
	E_AB  UMETA(DisplayName = "AB"),
	E_BA  UMETA(DisplayName = "BA"),
	E_B  UMETA(DisplayName = "B"),
	E_BC  UMETA(DisplayName = "BC"),
	E_CB  UMETA(DisplayName = "CB"),
	E_C  UMETA(DisplayName = "C"),
	E_CA  UMETA(DisplayName = "CA"),
	E_AC  UMETA(DisplayName = "AC"),
	E_NorthWest UMETA(DisplayName = "NorthWest"),
	E_NorthEast UMETA(DisplayName = "NorthEast"),
	E_SouthEast UMETA(DisplayName = "SouthEast"),
	E_SouthWest UMETA(DisplayName = "SouthWest"),
};


USTRUCT(BlueprintType)
struct MANUALNAVMESH_API FPCGTriCorners
{
	GENERATED_BODY()
public:
	bool NE;
	bool SE;
	bool SW;
	bool NW;

	bool IsContained()
	{
		return NE && SE && SW && NW;
	}
};

USTRUCT(BlueprintType)
struct MANUALNAVMESH_API FPCGTriPointMapping
{
	GENERATED_BODY()
public:
	int32 ParentPoint;
	int32 ParentEdge;

	FPCGTriPointMapping()
		: ParentPoint(-1)
		, ParentEdge(-1)
	{
	}

	FPCGTriPointMapping(int32 inPoint, int32 inEdge)
		: ParentPoint(inPoint)
		, ParentEdge(inEdge)
	{
	}
};

USTRUCT(BlueprintType)
struct MANUALNAVMESH_API FPCGTriPoint
{
	GENERATED_BODY()
public:
	int32 Index;
	EPCGTriPoint Point;

	FPCGTriPoint()
		: Index(-1)
		, Point(EPCGTriPoint::E_External)
	{
	}

	FPCGTriPoint(int32 inKey, EPCGTriPoint inValue)
		: Index(inKey)
		, Point(inValue)
	{
	}

	FString ToString() const;
};

UCLASS(notplaceable, Blueprintable)
class MANUALNAVMESH_API AManualDetourNavMesh : public ARecastNavMesh
{
	GENERATED_BODY()

public:
	AManualDetourNavMesh();

	UFUNCTION(BlueprintCallable, Category = "ManualRecastNavMesh")
	static AManualDetourNavMesh* GetManualRecastNavMesh(UObject* Context);

	dtNavMesh* NavMesh;
	dtNavMeshCreateParams params;
	FPImplRecastNavMesh* GetPirnm();

	UFUNCTION(BlueprintCallable, Category = "ManualRecastNavMesh")
	bool BuildManualNavMesh();

	bool SetNavMeshFromVerts(TArray<FVector>& Vertices, TArray<int32> &Indices);
	static FString DetourStatusToString(unsigned int status);
	static unsigned short DirectionToSide(EPCGDirection Border);
	unsigned char* BuildTileMesh(dtNavMesh* TiledNavMesh, const int32 tx, const int32 ty, const float* bmin, const float* bmax, int32 &NavDataSize);
	static void MakeInitParams(dtNavMeshParams& inParams, const float* bmin, float inTileSize);

	static FVector GetVector(dtNavMeshCreateParams& TileParams, unsigned short Index);
	static void SetDetourVector(unsigned short* verts, const FVector V, unsigned short Index, dtNavMeshCreateParams& TileParams);
	int32 GetTileCount() const;
	int32 GetGridSubdivisions() const;
	//FVector GetMin() const;
	//FVector GetMax() const;
	//bool GetGrid() const;

	static void MoveToCentre(FVector &InVector, const FVector Centre, float Distance = 1.f);
protected:
	void MakeGridTile(dtNavMeshCreateParams& TileParams, const int32 tx, const int32 ty);
	void MakeTriangulationTile(dtNavMeshCreateParams& TileParams, const int32 tx, const int32 ty, const FPCGDelaunayTriangulation& Triangulation);
	void DebugDrawTileParams(dtNavMeshCreateParams& TileParams);
	static FColor GetEdgeDebugColor(unsigned short EdgeCode);
	static FVector GetPolyCentre(dtNavMeshCreateParams& TileParams, unsigned short Index);

	bool IsBorder(unsigned short Adjacency);

	FString CornerString(FPCGTriCorners& Corners);

	// Create the triangle TriIndex
	int32 ProcessTriangle(const FBox2D& Bounds, int32 TriIndex, const FPCGDelaunayTriangulation &Triangulation,
		TArray<FVector2D> &TmpVerts, TArray<unsigned short> &TmpPolys, int32 CurrentPoly, TMap<int32, FPCGTriPoint> &InternalVerts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts);

	// Add relevant point to TmpVerts and mapping to InternalVerts, ignore points outside bounds
	FPCGTriPoint ProcessPoint(int32 PointIndex, const FBox2D& Bounds, const TArray<FVector2D>& Coords, TArray<FVector2D>& TmpVerts, TMap<int32, FPCGTriPoint>& InternalVerts);

	//typedef TTuple<FPCGTriPoint*, bool> TriPointType;
	//typedef TTuple<FPCGTriPoint*, FPCGTriPointMapping> TriPointType;
	typedef TTuple<FPCGTriPoint*, EPCGTriMember> TriPointType;
	typedef TArray<TriPointType, TInlineAllocator<6>> TriPointArray;
	//typedef TArray<TTuple<FPCGTriPoint*, bool>, TInlineAllocator<6>> TriPointArray;

	void PlacePoints(int32 PolyIndex, int32 TriIndex, TriPointArray& PointsToPlace, TArray<FVector2D>& TmpVerts, TArray<unsigned short>& TmpPolys, TMap<int32, FPCGTriPoint>& InternalVerts,
		TMap<int32, FPCGTriPoint>& HalfEdgeVerts, FPCGTriCorners& Corners, const FPCGDelaunayTriangulation& Triangulation, const FBox2D &Bounds);

	bool PlaceCorners(int32 &PlaceIndex, TriPointType& CurrentPoint, TriPointType& NextPoint, int32 PolyIndex, int32 TriIndex, TriPointArray& PointsToPlace, TArray<FVector2D>& TmpVerts, TArray<unsigned short>& TmpPolys, TMap<int32, FPCGTriPoint>& InternalVerts,
		TMap<int32, FPCGTriPoint>& HalfEdgeVerts, FPCGTriCorners& Corners, const FPCGDelaunayTriangulation& Triangulation, const FBox2D &Bounds);

	unsigned short GetAdjacentPoly(const FPCGDelaunayTriangulation& Triangulation, int32 TriIndex, TriPointType& CurrentPoint, TriPointType& NextPoint);
	FVector2D CornerEdgeToVert(EPCGTriPoint CornerEdge, const FBox2D& Bounds);
	EPCGTriMember GetConnectingEdge(EPCGTriMember P1, EPCGTriMember P2);
	unsigned short ConnectingEdgeToOffset(EPCGTriMember Edge);
	bool ReverseEdgeCheck(EPCGTriMember E1, EPCGTriMember E2);

	EPCGTriPoint GetNextEdge(EPCGTriPoint CurrentEdge,EPCGTriPoint StartEdge, EPCGTriPoint EndEdge, FPCGTriCorners &Corners);
	EPCGTriPoint GetNextCornerEdge(EPCGTriPoint Corner, EPCGTriPoint Destination, FPCGTriCorners &Corners);
	int32 CornerDistToEdge(EPCGTriPoint Corner, EPCGTriPoint Destination);

	FString TriPointToString(EPCGTriPoint TriPoint) const;
	FString TriMemberToString(EPCGTriMember TriMember) const;

	void ReversePolyVertOrder(int32 PolyIndex, int32 PolyVertCount, TArray<unsigned short>& TmpPolys);

	unsigned short GetEdgePortal(EPCGTriPoint Edge);
	EPCGTriPoint GetCommonEdge(EPCGTriPoint Edge1, EPCGTriPoint Edge2);
	bool IsEdge(EPCGTriMember Member);
	bool IsPoint(EPCGTriMember Member);

	void ProcessEdge(FPCGTriPoint &Edge1, FPCGTriPoint &Edge2, int32 EdgeIndex, const FPCGTriPoint &A, const FPCGTriPoint &B, const FBox2D& Bounds, const FPCGDelaunayTriangulation &Triangulation,
		TArray<FVector2D> &TmpVerts, TArray<unsigned short> &TmpPolys, int32 &CurrentPoly, TMap<int32, FPCGTriPoint> &InternalVerts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts);

	void ColinearEdgeCheck(FPCGTriPoint &Edge, int32 EdgeIndex, const FPCGTriPoint &PA, const FPCGTriPoint &PB, FVector2D &VA, FVector2D &VB, const FBox2D& Bounds, const FPCGDelaunayTriangulation &Triangulation,
		TArray<FVector2D> &TmpVerts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts);

	void IntersectingEdgeCheck(FPCGTriPoint &Edge, int32 EdgeIndex, const FPCGTriPoint &PA, const FPCGTriPoint &PB, FVector2D &VA, FVector2D &VB, const FBox2D& Bounds, const FPCGDelaunayTriangulation &Triangulation,
		TArray<FVector2D> &TmpVerts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts);

	bool TriangleRelevanceCheck(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FBox2D& Bounds);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	AWorldMap* SourceMap;

	float GetHeightAt(float X, float Y);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	float DebugThickness;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	float WalkableHeight;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	float WalkableRadius;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	float WalkableClimb;
	UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	bool bBuildBVTree;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	int32 TileCount;
	// If a grid is being used, how many times should it be subdivided within each tile
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	int32 GridSubdivisions;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	bool bNoPortals;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	bool bNoFlags;

	/*
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	bool bGrid;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	FVector Min;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	FVector Max;*/

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	bool bDebugDraw;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	FIntPoint DebugTile;

	// Controls the amount of temp space to reserve for points
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	int32 TempReserveVerts;
	// Controls the amount of temp space to reserve for polys 
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ManualDetour")
	int32 TempReservePolys;

	//UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	//float CellSize;
	//UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	//float CellHeight;


private:
	float TileSize;
	FVector Min;
	FVector Max;

	void AddHalfEdgeTempVert(FVector2D Location, FPCGTriPoint& Point, int32 EdgeIndex, EPCGTriPoint Type, TArray<FVector2D> &Verts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts);
	bool IsInternalSegment(const FPCGTriPoint& A, const FPCGTriPoint& B);

	int32 GetHalfEdge(int32 Index, EPCGTriMember Member, const FPCGDelaunayTriangulation &Triangulation);

	void LogTriPoly(int32 Index, FPCGTriPoint &EdgeAB, FPCGTriPoint &EdgeBA,FPCGTriPoint &EdgeBC,FPCGTriPoint &EdgeCB,FPCGTriPoint &EdgeCA,FPCGTriPoint &EdgeAC,
		FPCGTriPoint &A, FPCGTriPoint &B, FPCGTriPoint &C, FPCGTriCorners& Corners, const FPCGDelaunayTriangulation &Triangulation);
	void LogPlacedPoly(int32 Index, TArray<unsigned short>& Polys);

	int32 PointCount(FPCGTriPoint& EdgeAB, FPCGTriPoint& EdgeBA, FPCGTriPoint& EdgeBC, FPCGTriPoint& EdgeCB, FPCGTriPoint& EdgeCA, FPCGTriPoint& EdgeAC, FPCGTriPoint& A, FPCGTriPoint& B, FPCGTriPoint& C, FPCGTriCorners& Corners);
	int32 PointAccum(FPCGTriPoint& EdgeAB, FPCGTriCorners& Corners);
};
	
