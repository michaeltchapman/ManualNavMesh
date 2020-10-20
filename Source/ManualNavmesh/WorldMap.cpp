// Fill out your copyright notice in the Description page of Project Settings.


#include "WorldMap.h"
#include "ProceduralMeshComponent.h"
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

	FVector Min = NavMesh->GetMin();
	FVector Max = NavMesh->GetMax();

	int32 Subdivisions = NavMesh->GetTileCount() * NavMesh->GetGridSubdivisions() + 1;

	float Spacing = (Max.X - Min.X) / ((float)(Subdivisions - 1));
	float MeshHeight = Max.Z - Min.Z;

	if (Mesh->GetNumSections() > 0)
	{
		Mesh->ClearMeshSection(0);
	}

	TArray<int32> Triangles;
	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<FProcMeshTangent> Tangents;
	TArray<FVector2D> UVs;
	TArray<FColor> Colors;
	UKismetProceduralMeshLibrary::CreateGridMeshWelded(Subdivisions, Subdivisions, Triangles, Vertices, UVs, Spacing);

	for (auto& Vert : Vertices)
	{
		Vert.Z = Min.Z + MeshHeight*GetHeightAt(Vert.X, Vert.Y);
	}

	UKismetProceduralMeshLibrary::CalculateTangentsForMesh(Vertices, Triangles, UVs, Normals, Tangents);
	Colors.AddZeroed(Vertices.Num());
	Mesh->CreateMeshSection(0, Vertices, Triangles, Normals, UVs, Colors, Tangents, true);
	Mesh->SetMaterial(0, MeshMaterial);
}



