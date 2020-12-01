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
	Scale = 1.f;
	DebugHeight = 0.f;

	ObstacleChunks = 4;
	ObstacleProbability = 0.75f;
	ObstacleOuterRatio = 0.7f;
	ObstacleInnerRatio = 0.3f;
	ObstacleOuterHeight = 0.3f;
	ObstacleInnerHeight = 1.f;
	ObstacleMaxVerts = 4;

	bUseTPAStar = false;

}

FPCGDelaunayTriangulation& AWorldMap::GetTPATriangulation()
{
	return MinimumTriangulation;
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
	if (DebugHeight != 0.f)
	{
		return DebugHeight;
	}

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

	float MeshHeight = GetMax().Z - GetMin().Z;

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
		float Spacing = (GetMax().X - GetMin().X) / ((float)(Subdivisions - 1));
		UKismetProceduralMeshLibrary::CreateGridMeshWelded(Subdivisions, Subdivisions, MeshTriangles, MeshVertices, MeshUVs, Spacing);

		for (auto& Vert : MeshVertices)
		{
			Vert.Z = GetMin().Z + MeshHeight * GetHeightAt(Vert.X, Vert.Y);
		}
	}
	else {
		CoordHeights.Reset();
		Triangulation.Reset();

		TArray<FVector2D> Points;
		if (bUseTPAStar)
		{
			// 2D Triangulation
			GenerateObstacles(ObstacleChunks, Points, CoordHeights, true);
			URegionDistribution::GenerateTriangulation(Points, MinimumTriangulation);
			GenerateImpassableObstacles(MinimumTriangulation);

			PathingFlags.Flags.AddZeroed(MinimumTriangulation.Triangles.Num() / 3);
			FMemory::Memset(PathingFlags.Flags.GetData(),1, sizeof(int32) * MinimumTriangulation.Triangles.Num() / 3);

			MinimumTriangulation.Flags = (void*)&PathingFlags;
			for (auto Tri : ImpassableRegions)
			{
				//MinimumTriangulation.DeepRemoveTriangle(Tri);
				PathingFlags.Flags[Tri] = 0;
			}
			//ImpassableRegions.Reset();
			//MinimumTriangulation.CleanupRemovals();
			MinimumTriangulation.LogDumpToCSharpFormat(ImpassableRegions);

			for (int32 i = 0; i < MinimumTriangulation.Triangles.Num() / 3; i++)
			{
				if (PathingFlags.CanTraverse(i))
				{
					URegionDistribution::DrawTriangle(this, i, MinimumTriangulation, 2.f, i);
				}
				else
				{
					URegionDistribution::DrawTriangle(this, i, MinimumTriangulation, 2.f, i, FColor::Red);
				}
			}


			// Heightmap Triangulation
			GenerateObstacles(ObstacleChunks, Points, CoordHeights, bSetFlatObstacles);
			URegionDistribution::GenerateTriangulation(Points, Triangulation);
			GenerateImpassableObstacles(Triangulation);

		}
		else {
			if (ObstacleChunks <= 0)
			{
				URegionDistribution::GenerateBoundedRandomPoints(RandomPointCount, FVector2D(GetMin()), FVector2D(GetMax()), Points, Seed);
			}
			else {
				GenerateObstacles(ObstacleChunks, Points, CoordHeights, bSetFlatObstacles);
			}

			URegionDistribution::GenerateTriangulation(Points, Triangulation);
			GenerateImpassableObstacles(Triangulation);
		}

		Triangulation.CleanupRemovals();

		MeshTriangles = Triangulation.Triangles;
		MeshVertices.Reserve(Triangulation.Coords.Num());
		MeshUVs.Reserve(Triangulation.Coords.Num());
		for (int32 i = 0; i < Triangulation.Coords.Num(); i++)
		{
			const FVector2D& V = Triangulation.Coords[i];
			CoordHeights.Add(GetMin().Z + MeshHeight*GetHeightAt(V.X, V.Y));
			MeshVertices.Add(FVector(Triangulation.Coords[i], CoordHeights[i]));
			MeshUVs.Add(Triangulation.Coords[i]);
		}
	}

	UKismetProceduralMeshLibrary::CalculateTangentsForMesh(MeshVertices, MeshTriangles, MeshUVs, MeshNormals, MeshTangents);
	MeshColors.AddZeroed(MeshVertices.Num());
	Mesh->CreateMeshSection(0, MeshVertices, MeshTriangles, MeshNormals, MeshUVs, MeshColors, MeshTangents, true);
	Mesh->SetMaterial(0, MeshMaterial);
}

FVector AWorldMap::GetMin() const
{
	return Min * Scale;
}

FVector AWorldMap::GetMax() const
{
	return Max * Scale;
}

bool AWorldMap::IsGrid() const
{
	return bGrid;
}

const TSet<int32>& AWorldMap::GetImpassableRegions() const
{
	return ImpassableRegions;
}

void AWorldMap::GenerateObstacles(int32 Chunks, TArray<FVector2D>& Points, TArray<float>& Heights, bool inbSetFlatObstacles)
{
	if (Chunks == 0)
		return;

	/*
	// Defaults
	ObstacleChunks = 4;
	ObstacleProbability = 0.75f;
	ObstacleOuterRatio = 0.7f;
	ObstacleInnerRatio = 0.3f;
	ObstacleOuterHeight = 0.3f;
	ObstacleInnerHeight = 1.f;
	ObstacleMaxVerts = 4;
	*/

	FVector WorldSize = (GetMax() - GetMin());
	FVector2D ChunkSize = FVector2D(WorldSize) / ((float)Chunks);
	FVector2D HalfChunkSize = ChunkSize * 0.5f;

	FRandomStream Random = FRandomStream(Seed);

	Points.Reset();
	Heights.Reset();

	Points.Add(FVector2D(GetMin()));
	CoordHeights.Add(GetMin().Z);
	Points.Add(FVector2D(GetMax()));
	CoordHeights.Add(GetMin().Z);
	Points.Add(FVector2D(GetMin().X, GetMax().Y));
	CoordHeights.Add(GetMin().Z);
	Points.Add(FVector2D(GetMax().X, GetMin().Y));
	CoordHeights.Add(GetMin().Z);

	DenyVerts.Reset();

	for (int32 j = 0; j < Chunks; j++)
	{
		for (int32 i = 0; i < Chunks; i++)
		{
			if (Random.GetFraction() < ObstacleProbability)
			{
				FVector2D ChunkMin = FVector2D(GetMin().X + ChunkSize.X * i,GetMin().Y + ChunkSize.Y * j);
				FVector2D ChunkMax = ChunkMin + ChunkSize;
				FVector2D ChunkCentre = ChunkMin + HalfChunkSize;

				int32 VertCount = Random.RandRange(4, ObstacleMaxVerts);
				float RotateDegrees = 360.f / ((float)VertCount);

				DenyVerts.Add(Points.Num());
				Points.Add(ChunkCentre);
				CoordHeights.Add(GetMin().Z + WorldSize.Z*ObstacleInnerHeight);
				for (int32 v = 0; v < VertCount; v++)
				{
					FVector2D Outer = FVector2D(FVector2D::Unit45Deg).GetRotated(v * RotateDegrees) * HalfChunkSize * ObstacleOuterRatio + ChunkCentre;
					FVector2D Inner = FVector2D(FVector2D::Unit45Deg).GetRotated(v * RotateDegrees) * HalfChunkSize * ObstacleInnerRatio + ChunkCentre;

					if (inbSetFlatObstacles)
					{
						Points.Add(Inner);
						CoordHeights.Add(GetMin().Z + WorldSize.Z * ObstacleOuterHeight);
					}
					else {
						Points.Add(Outer);
						CoordHeights.Add(GetMin().Z);
						Points.Add((Outer + Inner) * 0.5f);
						CoordHeights.Add(GetMin().Z + WorldSize.Z * ObstacleOuterHeight);
						Points.Add(Inner);
						CoordHeights.Add(GetMin().Z + WorldSize.Z * ObstacleOuterHeight);
					}
				}
			}
		}
	}
}

void AWorldMap::GenerateImpassableObstacles(FPCGDelaunayTriangulation &inTriangulation)
{
	ImpassableRegions.Reset();
	for (int32 i = 0; i < inTriangulation.Triangles.Num(); i++)
	{
		for (auto Vert : DenyVerts)
		{
			if (inTriangulation.Triangles[i] == Vert)
			{
				ImpassableRegions.Add(inTriangulation.TriangleOfEdge(i));
			}
		}
	}
}
