// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FastNoise/FastNoise.h"
#include "RegionDistribution.h"
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

	UPROPERTY()
	FPCGDelaunayTriangulation Triangulation;
	/*UPROPERTY()
	TArray<FVector2D> Coords;
	UPROPERTY()
	TArray<int32> Triangles;
	UPROPERTY()
	TArray<int32> HalfEdges;*/

	FVector GetMin() const;
	FVector GetMax() const;
	bool IsGrid() const;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	int32 RandomPointCount;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	bool bGrid;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	FVector Min;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "WorldMap")
	FVector Max;


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

public:	

	UFUNCTION(BlueprintCallable, Category = "WorldMap")
	void BuildMesh();
};
