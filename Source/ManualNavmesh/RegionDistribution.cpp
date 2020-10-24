#include "RegionDistribution.h"
#include "ManualDetourNavMesh.h"
#include "DrawDebugHelpers.h"
#include "Delaunator.h"

void URegionDistribution::GenerateBoundedRandomPoints(int32 Count, FVector2D Min, FVector2D Max, UPARAM(ref) TArray<FVector2D>& OutPoints, int32 Seed)
{
	FRandomStream Random = FRandomStream(Seed);

	OutPoints.Reset(Count);

	OutPoints.Add(Min);
	OutPoints.Add(FVector2D(Min.X, Max.Y));
	OutPoints.Add(Max);
	OutPoints.Add(FVector2D(Max.X, Min.Y));

	for (int32 i = 0; i < Count; i++)
	{
		OutPoints.Add(FVector2D(Random.FRandRange(Min.X, Max.X), Random.FRandRange(Min.Y, Max.Y)));
	}

}

void URegionDistribution::GenerateTriangulation(UPARAM(ref) TArray<FVector2D>& inPoints, UPARAM(ref) TArray<FVector2D>& OutVerts, UPARAM(ref) TArray<int32>& Triangles, UPARAM(ref) TArray<int32>& HalfEdges)
{
	std::vector<double> TestCoords;
	TestCoords.reserve(inPoints.Num() * 2);
	for (auto& V : inPoints)
	{
		TestCoords.push_back(V.X);
		TestCoords.push_back(V.Y);
	}

	delaunator::Delaunator Delaunator(TestCoords);

	OutVerts.Reset(Delaunator.coords.size() / 2);
	for (int32 i = 0; i < Delaunator.coords.size(); i++)
	{
		OutVerts.Add(FVector2D(Delaunator.coords[2 * i], Delaunator.coords[2 * i + 1]));
	}
	Triangles.Reset(Delaunator.coords.size());
	HalfEdges.Reset(Delaunator.coords.size());
	for (int32 i = 0; i < Delaunator.triangles.size(); i++)
	{
		Triangles.Add(Delaunator.triangles[i]);
		HalfEdges.Add(Delaunator.halfedges[i]);
	}
}

void URegionDistribution::DrawTriangle(UObject* WorldContextObject, int32 Index, const TArray<FVector2D>& OutVerts, const TArray<int32>& Triangles, const TArray<int32>& HalfEdges, float Inset)
{
	if (WorldContextObject)
	{
		auto* World = WorldContextObject->GetWorld();
		if (World)
		{
			int32 TriIndex = Index * 3;

			if (Triangles.IsValidIndex(TriIndex))
			{

				FVector A = FVector(OutVerts[Triangles[TriIndex]], 0.f);
				FVector B = FVector(OutVerts[Triangles[TriIndex + 1]], 0.f);
				FVector C = FVector(OutVerts[Triangles[TriIndex + 2]], 0.f);

				FVector Centre = (A + B + C) / 3.f;
				AManualDetourNavMesh::MoveToCentre(A, Centre, Inset);
				AManualDetourNavMesh::MoveToCentre(B, Centre, Inset);
				AManualDetourNavMesh::MoveToCentre(C, Centre, Inset);

				DrawDebugLine(World, A, B, FColor::White, true, 999.f, 0, Inset * 0.5f);
				DrawDebugLine(World, C, B, FColor::White, true, 999.f, 0, Inset * 0.5f);
				DrawDebugLine(World, A, C, FColor::White, true, 999.f, 0, Inset * 0.5f);
			}
		}
	}
}
