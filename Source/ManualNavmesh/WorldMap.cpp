// Fill out your copyright notice in the Description page of Project Settings.


#include "WorldMap.h"
#include "ProceduralMeshComponent.h"
#include "RegionDistribution.h"
#include "ManualDetourNavMesh.h"
#include "KismetProceduralMeshLibrary.h"
//#include "FastNoise/FastNoise.h"

// Sets default values
AWorldMap::AWorldMap()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
	Mesh->bUseComplexAsSimpleCollision;
	Mesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Mesh->SetCollisionProfileName(FName(TEXT("BlockAll")));
}

// Called when the game starts or when spawned
void AWorldMap::BeginPlay()
{
	Super::BeginPlay();
	MakeNoise();
}

void AWorldMap::MakeNoise()
{
	if (!Noise)
	{
		Noise = NewObject<UFastNoise>(this);
	}

	Noise->SetNoiseType(NoiseType);
	Noise->SetSeed(Seed);
	Noise->SetFractalOctaves(Octaves);
	Noise->SetFrequency(Frequency);
	Noise->SetFractalType(FractalType);
	Noise->SetFractalLacunarity(Lacunarity);
	Noise->SetFractalGain(FractalGain);
	Noise->SetCellularDistanceFunction(CellularDistanceFunction);
	Noise->SetCellularReturnType(CellularReturnType);
	Noise->SetInterp(Interpolation);

}

float AWorldMap::GetHeightAt(float X, float Y)
{
	if (Noise)
	{
		return (Noise->GetNoise2D(X, Y) + 1.f) * 0.5f;
	}
	return 0.f;
}

void AWorldMap::BuildMesh()
{
	AManualDetourNavMesh* NavMesh = AManualDetourNavMesh::GetManualRecastNavMesh(this);
	if (!NavMesh)
	{
		return;
	}

	MakeNoise();

	float MeshHeight = Max.Z - Min.Z;

	if (Mesh->GetNumSections() > 0)
	{
		Mesh->ClearMeshSection(0);
	}

	TArray<int32> MeshTriangles;
	TArray<FVector> MeshVertices;
	TArray<FVector> MeshNormals;
	TArray<FProcMeshTangent> MeshTangents;
	TArray<FVector2D> MeshUVs;
	TArray<FColor> MeshColors;

	if (bGrid)
	{
		int32 Subdivisions = NavMesh->GetTileCount() * NavMesh->GetGridSubdivisions() + 1;
		float Spacing = (Max.X - Min.X) / ((float)(Subdivisions - 1));
		UKismetProceduralMeshLibrary::CreateGridMeshWelded(Subdivisions, Subdivisions, MeshTriangles, MeshVertices, MeshUVs, Spacing);
	}
	else {
		TArray<FVector2D> Points;
		URegionDistribution::GenerateBoundedRandomPoints(RandomPointCount, FVector2D(Min), FVector2D(Max), Points);
		//URegionDistribution::GenerateTriangulation(Points, Coords, Triangles, HalfEdges);

		URegionDistribution::GenerateTriangulation(Points, Triangulation);

		MeshTriangles = Triangulation.Triangles;
		MeshVertices.Reserve(Triangulation.Coords.Num());
		MeshUVs.Reserve(Triangulation.Coords.Num());
		for (int32 i = 0; i < Triangulation.Coords.Num(); i++)
		{
			MeshVertices.Add(FVector(Triangulation.Coords[i], 0.f));
			MeshUVs.Add(Triangulation.Coords[i]);
		}
	}

	for (auto& Vert : MeshVertices)
	{
		Vert.Z = Min.Z + MeshHeight*GetHeightAt(Vert.X, Vert.Y);
	}

	UKismetProceduralMeshLibrary::CalculateTangentsForMesh(MeshVertices, MeshTriangles, MeshUVs, MeshNormals, MeshTangents);
	MeshColors.AddZeroed(MeshVertices.Num());
	Mesh->CreateMeshSection(0, MeshVertices, MeshTriangles, MeshNormals, MeshUVs, MeshColors, MeshTangents, true);
	Mesh->SetMaterial(0, MeshMaterial);
}

FVector AWorldMap::GetMin() const
{
	return Min;
}

FVector AWorldMap::GetMax() const
{
	return Max;
}

bool AWorldMap::IsGrid() const
{
	return bGrid;
}
