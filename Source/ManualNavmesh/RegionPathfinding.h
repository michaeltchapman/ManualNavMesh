// Copyright Michael Chapman 2020
#pragma once

#include "Engine.h"
#include "CoreMinimal.h"
//#include "UObject/NoExportTypes.h"
#include "GameplayTagContainer.h"
#include "RegionDistribution.h"

#include "RegionPathfinding.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogUPCGTRegionPathfinding, Log, All);

USTRUCT()
struct MANUALNAVMESH_API FPCGTriangleEvalResult
{
	GENERATED_BODY()

	FPCGTriangleEvalResult()
		: EstimatedMinimalCost(0.f)
		, ShortestPathToEdgeLength(0.f)
		, LongestPathToEdgeLength(0.f)
	{}
	
	FPCGTriangleEvalResult(float inEstimatedMinimalCost,float inShortestPathToEdgeLength,float inLongestPathToEdgeLength)
		: EstimatedMinimalCost(inEstimatedMinimalCost)
		, ShortestPathToEdgeLength(inShortestPathToEdgeLength)
		, LongestPathToEdgeLength(inLongestPathToEdgeLength)
	{}
	
	UPROPERTY()
	float EstimatedMinimalCost;
	UPROPERTY()
	float ShortestPathToEdgeLength;
	UPROPERTY()
	float LongestPathToEdgeLength;
};

UENUM(BlueprintType)
enum class EPCGFunnelSide : uint8
{
	E_Left UMETA(DisplayName = "Left"),
	E_Right UMETA(DisplayName = "Right"),
	E_Both UMETA(DisplayName = "Both"),
	E_None UMETA(DisplayName = "None")
};

USTRUCT()
struct MANUALNAVMESH_API FPCGPathFunnel
{
	GENERATED_BODY()
public:
	FPCGPathFunnel()
		: Apex()
		, Triangulation(nullptr)
	{};
	
	FPCGPathFunnel(FVector2D StartPoint, FPCGDelaunayTriangulation *inTriangulation);
	FPCGPathFunnel(const FPCGPathFunnel &Other);
	FPCGPathFunnel& operator=(const FPCGPathFunnel& Other);
	
	TDoubleLinkedList<FVector2D> Funnel;
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Apex;
	TDoubleLinkedList<FVector2D> Path;


	// Extend the funnel putting a new edge at the end
	void StepOver(int32 Edge);
	
	// Cleans up the funnel and adds the required points to the path to define a complete path
	void FinalisePath(FVector2D Goal);

	void InitFunnel(int32 FirstEdge);
	EPCGFunnelSide DetermineSideSharedBy(int32 Edge, TDoubleLinkedList<FVector2D> &Funnel);
	void AddToRightSideOfFunnel(FVector2D Point);
	void AddToLeftSideOfFunnel(FVector2D Point);
	FPCGDelaunayTriangulation* Triangulation;

	void DebugDraw(FVector2D Offset = FVector2D::ZeroVector) const;
	void DebugLog() const;
	FString ToString() const;
};


USTRUCT()
struct MANUALNAVMESH_API FPCGTPAPath
{
	GENERATED_BODY()
	
	
public:

	FPCGTPAPath()
		: bReachedPathsBuilt(false)
		, CurrentTriangle(-1)
		, CurrentEdge(-1)
		, AlreadyBuiltPathLength(0.f)
		, LengthOfShortestPathFromApexToEdge(0.f)
		, LengthOfLongestPathFromApexToEdge(0.f)
		, DistanceFromClosestGoalPoint(0.f)
		, Triangulation(nullptr)
	{
	}

	FPCGTPAPath(FVector2D StartPoint, int32 StartTriangle, FPCGDelaunayTriangulation *inTriangulation)
		: bReachedPathsBuilt(false)
		, CurrentTriangle(StartTriangle)
		, CurrentEdge(-1)
		, Funnel(StartPoint, inTriangulation)
		, AlreadyBuiltPathLength(0.f)
		, LengthOfShortestPathFromApexToEdge(0.f)
		, LengthOfLongestPathFromApexToEdge(0.f)
		, DistanceFromClosestGoalPoint(0.f)
		, Triangulation(inTriangulation)
	{
	}
	
	//FPCGTPAPath(const FPCGTPAPath& Other);


	float ShortestPathToEdgeLength() const;
	float LongestPathToEdgeLength() const;
	float MinimalTotalCost() const;
	bool bReachedPathsBuilt;
	FPCGTPAPath BuildPartialPathTo(int32 Neighbour, FVector2D Goal);
	void BuildCompletePathTo(FVector2D Goal, TDoubleLinkedList<FVector2D> &OutPath);
	void UpdateEstimationToClosestGoalPoint(FVector2D Goal);
	void StepTo(int32 TargetTriangle, FVector2D Goal);
	void PathToArray(TArray<FVector2D>& OutArray);
	FPCGTPAPath Clone();

	// these are static in the reference but we need the triangulation ptr to do almost anything 
	float GetLengthOfShortestPathFromApexToEdge(int32 Edge, const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Apex);
	float GetLengthOfLongestPathFromApexToEdge(TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Apex);
	float WalkOnRightSideOfFunnelUntilClosestPointBecomesVisible(const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* StartNode, int32 Edge);
	float WalkOnLeftSideOfFunnelUntilClosestPointBecomesVisible(const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* StartNode, int32 Edge);
	float LengthOfBuiltPathInFunnel(const TDoubleLinkedList<FVector2D> &Path);
	float MinimalDistanceBetween(int32 Edge, FVector2D TargetPoint, bool bShouldIncludePointsInTriangle, int32 Triangle);
	float MinimalDistanceBetween(FVector2D Point, FVector2D TargetPoint, bool bShouldIncludePointsInTriangle, int32 Triangle);
	int32 GetCurrentTriangle() const;
	int32 GetCurrentEdge() const;

	void DebugDraw(FVector2D Offset = FVector2D::ZeroVector) const;
	void DebugLog() const;
	FString ToString() const;
	
private:
	int32 CurrentTriangle;
	int32 CurrentEdge;
	FPCGPathFunnel Funnel;
	float AlreadyBuiltPathLength;
	float LengthOfShortestPathFromApexToEdge;
	float LengthOfLongestPathFromApexToEdge;
	float DistanceFromClosestGoalPoint;
	FPCGDelaunayTriangulation* Triangulation;
};

USTRUCT()
struct MANUALNAVMESH_API FPCGTPAPathFinder
{
	GENERATED_BODY()

	FPCGTPAPathFinder()
	{
	}
	
	FPCGTPAPathFinder(FPCGDelaunayTriangulation *inTriangulation)
		: Triangulation(inTriangulation)
	{
	}
	
	FPCGTPAPathFinder& operator=(const FPCGTPAPathFinder& Other);

	//bool FindPath(const FVector2D &StartPoint, int32 StartTriangle, const FVector2D &Goal, TDoubleLinkedList<FVector2D> &OutPath);
	bool FindPath(const FVector2D &StartPoint, int32 StartTriangle, const FVector2D &Goal, TArray<FVector2D> &OutPath);
	float GetLength(const TDoubleLinkedList<FVector2D> &Path);
	bool IsGoodCandidate(const FPCGTPAPath &Path) const;
	void UpdateHigherBoundToReachedEdge(const FPCGTPAPath &Path);
	void AddToOpenSet(const FPCGTPAPath &Path);

	static void CloneDoubleLL(const TDoubleLinkedList<FVector2D> &From, TDoubleLinkedList<FVector2D> &To);

	void DebugDraw(FVector2D Offset = FVector2D::ZeroVector) const;
	void DebugLog() const;
	void SetTriangulation(FPCGDelaunayTriangulation *Triangulation);
private:
	TDoubleLinkedList<FPCGTPAPath> OpenSet;
	UPROPERTY()
	TMap<int32, float> HigherBounds;

	FPCGDelaunayTriangulation* Triangulation;
	
};

