// Copyright Michael Chapman 2018
#pragma once

#include "CoreMinimal.h"
#include "Navmesh/RecastNavMesh.h"
#include "Detour/DetourNavMeshBuilder.h"
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
	void MakeTriangulationTile(dtNavMeshCreateParams& TileParams, const int32 tx, const int32 ty, const TArray<FVector2D> &Coords, const TArray<int32> &Triangles, const TArray<int32> &HalfEdges);
	void DebugDrawTileParams(dtNavMeshCreateParams& TileParams);
	static FColor GetEdgeDebugColor(unsigned short EdgeCode);
	static FVector GetPolyCentre(dtNavMeshCreateParams& TileParams, unsigned short Index);

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

	//UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	//float CellSize;
	//UPROPERTY(BlueprintReadWrite, EditDefaultsOnly, Category = "Detour")
	//float CellHeight;

	bool BoundsIntersectsSegment(const FBox2D& Bounds, const FVector2D& A, const FVector2D& B);
	bool SegmentIntersectionTest(const FVector2D& SegmentStartA, const FVector2D& SegmentEndA, const FVector2D& SegmentStartB, const FVector2D& SegmentEndB);

private:
	float TileSize;
	FVector Min;
	FVector Max;

};
	
