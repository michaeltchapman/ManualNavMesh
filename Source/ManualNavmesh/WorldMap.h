// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FastNoise/FastNoise.h"
#include "RegionDistribution.h"
#include "RegionPathfinding.h"
#include "WorldMap.generated.h"

class UProceduralMeshComponent;

UCLASS(Blueprintable)
class MANUALNAVMESH_API AWorldMap : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AWorldMap();

	float GetHeightAt(float X, float Y);
	void MakeNoise();

	// triangulation including height changes upon which paths are mapped
	UPROPERTY()
	FPCGDelaunayTriangulation Triangulation;
	UPROPERTY()
	TArray<float> CoordHeights;

	FVector GetMin() const;
	FVector GetMax() const;
	bool IsGrid() const;

	const TSet<int32>& GetImpassableRegions() const;

	FPCGDelaunayTriangulation& GetTPATriangulation();
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	TSet<int32> ImpassableRegions;

	// Minimal triangulation used for pathfinding using TPAStar
	UPROPERTY()
	FPCGDelaunayTriangulation MinimumTriangulation;

	// A traversability flag for each tri, > 0 means traversable
	UPROPERTY()
	FPCGPathfindingFlags PathingFlags;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	int32 RandomPointCount;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	bool bGrid;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	FVector Min;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	FVector Max;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	float Scale;

	// How many obstacle regions will be created
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	int32 ObstacleChunks;

	// The chance for any given chunk to contain an obstacle
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	float ObstacleProbability;

	// How much of the region can be taken up by the outer obstacle
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	float ObstacleOuterRatio;

	// How much of the region can be taken up by the inner obstacle
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	float ObstacleInnerRatio;

	// How high the outer obstacle points will be
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	float ObstacleOuterHeight;

	// How high the inner obstacle points will be
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	float ObstacleInnerHeight;

	// How many points the obstacle can have. Minimum is 3
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	int32 ObstacleMaxVerts;

	// Whether to have heights for obstacles or simulate a flat map
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	bool bSetFlatObstacles;

	// Whether to remove impassable triangles from the triangulation by setting
	// them to have no points and no edges, and removing the edges from adjacents.
	// Will not change the memory allocation so consumers will need to check
	// ValidTriangle() before operating on each triangle
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	bool bDeleteObstacleTriangles;

	// Whether to remove impassable triangles from the triangulation by setting
	// them to have no points and no edges, and removing the edges from adjacents.
	// A second navmesh will be created with the height changes included that can
	// be projected upon using the TPAS path
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	bool bUseTPAStar;

	void GenerateObstacles(int32 Chunks, TArray<FVector2D>& Points, TArray<float>& Heights, bool SetFlatObstacles);
	void GenerateImpassableObstacles(FPCGDelaunayTriangulation &inTriangulation);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapMesh")
	float DebugHeight;
	UPROPERTY()
	UFastNoise* Noise;

	UPROPERTY()
	UProceduralMeshComponent* Mesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapMesh")
	UMaterialInterface* MeshMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	int Seed = 1337;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	float Frequency = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	EInterp Interpolation = EInterp::InterpQuintic;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	ENoiseType NoiseType = ENoiseType::Simplex;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	EPositionWarpType PositionWarpType = EPositionWarpType::None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	int Octaves = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	float Lacunarity = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	float FractalGain = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	EFractalType FractalType = EFractalType::FBM;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	ECellularDistanceFunction CellularDistanceFunction = ECellularDistanceFunction::Euclidean;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "WorldMapNoise")
	ECellularReturnType CellularReturnType = ECellularReturnType::CellValue;

	UPROPERTY()
	TArray<int32> DenyVerts;
public:	

	UFUNCTION(BlueprintCallable, Category = "WorldMap")
	void BuildMesh();
};
