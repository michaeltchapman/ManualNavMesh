#pragma once

#include "CoreMinimal.h"
#include "RegionDistribution.generated.h"

USTRUCT(BlueprintType)
struct MANUALNAVMESH_API FPCGDelaunayTriangulation
{
	GENERATED_BODY()
public:

	FORCEINLINE static FIntVector EdgesOfTriangle(int32 Index) { return FIntVector(3 * Index, 3 * Index + 1, 3 * Index + 2); };
	FORCEINLINE static int32 TriangleOfEdge(int32 Index) { return Index < 0 ? Index : FMath::FloorToInt(Index / 3); };
	FORCEINLINE static int32 NextHalfedge(int32 Edge) { return (Edge % 3 == 2) ? Edge - 2 : Edge + 1; };
	FORCEINLINE static int32 PrevHalfedge(int32 Edge) { return (Edge % 3 == 0) ? Edge + 2 : Edge - 1; };

	FORCEINLINE bool TriangleHasPoint(int32 Triangle, int32 Point)
	{
		for (int32 i = 3 * Triangle; i < 3 * Triangle + 3; i++)
		{
			if (Triangles[i] == Point) { return true; }
		} return false;
	};

	FORCEINLINE int32 HalfEdgeOrigin(const int32 Edge) { return Triangles[Edge]; };
	FORCEINLINE int32 HalfEdgeDest(int32 Edge) { (Edge % 3 == 2) ? Edge -= 2 : Edge += 1; return Triangles[Edge]; };

	UPROPERTY()
	TArray<FVector2D> Coords;
	UPROPERTY()
	TArray<int32> Triangles;
	UPROPERTY()
	TArray<int32> HalfEdges;
};

UCLASS()
class MANUALNAVMESH_API URegionDistribution : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
public:
	UFUNCTION(BlueprintCallable, Category = "RegionDistribution")
	static void GenerateBoundedRandomPoints(int32 Count, FVector2D Min, FVector2D Max, UPARAM(ref) TArray<FVector2D>& OutPoints, int32 Seed = 0);
	//static void GenerateTriangulation(UPARAM(ref) TArray<FVector2D>& inPoints, UPARAM(ref) TArray<FVector2D>& OutVerts, UPARAM(ref) TArray<int32>& Triangles, UPARAM(ref) TArray<int32>& HalfEdges);

	UFUNCTION(BlueprintCallable, Category = "RegionDistribution")
	static void GenerateTriangulation(UPARAM(ref) TArray<FVector2D>& inPoints, UPARAM(ref) FPCGDelaunayTriangulation& Triangulation);

	static void DrawTriangle(UObject* WorldContextObject, int32 Index, const FPCGDelaunayTriangulation& Triangulation, float Inset, int32 Label);

	static bool SegmentIntersectionTest(const FVector2D& SegmentStartA, const FVector2D& SegmentEndA, const FVector2D& SegmentStartB, const FVector2D& SegmentEndB);
	static bool IsInTriangle(FVector2D P, const FVector2D& A, const FVector2D& B, const FVector2D& C);
	static FVector GetBaryCentric2D(const FVector2D& Point, const FVector2D& A, const FVector2D& B, const FVector2D& C);

	static bool BoundsIntersectsSegment(const FBox2D& Bounds, const FVector2D& A, const FVector2D& B);
	static int32 SegmentBoundsIntersections(const FBox2D& Bounds, const FVector2D& A, const FVector2D& B, const FVector2D& C);
	static bool IsBoundsCorner(const FBox2D& Bounds, const FVector2D& V);

	// If AB is colinear with any of the segments that comprise the box Bounds, how many points lie on the edge
	// - Don't count corners if A or B is == the corner
	// - return 0 if not colinear with any segments
	// - return 1 if either A or B is on the edge but isn't a corner
	// - return 2 if both A and B are on the edge but isn't a corner
	static int32 SegmentColinearWithBoundsPoints(const FBox2D& Bounds, const FVector2D& A, const FVector2D& B, bool& bColinear);
	static int32 SegmentInternalColinearityCount(const FVector2D& Q, const FVector2D& R, const FVector2D& A, const FVector2D& B, bool bDirection, bool& bColinear);
};
