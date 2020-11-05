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

void URegionDistribution::GenerateTriangulation(UPARAM(ref) TArray<FVector2D>& inPoints, UPARAM(ref) FPCGDelaunayTriangulation& Triangulation)
{
	std::vector<double> TestCoords;
	TestCoords.reserve(inPoints.Num() * 2);
	for (auto& V : inPoints)
	{
		TestCoords.push_back(V.X);
		TestCoords.push_back(V.Y);
	}

	delaunator::Delaunator Delaunator(TestCoords);

	Triangulation.Coords.Reset(Delaunator.coords.size() / 2);
	for (int32 i = 0; i < Delaunator.coords.size() / 2; i++)
	{
		Triangulation.Coords.Add(FVector2D(Delaunator.coords[2 * i], Delaunator.coords[2 * i + 1]));
	}
	Triangulation.Triangles.Reset(Delaunator.triangles.size());
	Triangulation.HalfEdges.Reset(Delaunator.triangles.size());
	for (int32 i = 0; i < Delaunator.triangles.size(); i++)
	{
		Triangulation.Triangles.Add(Delaunator.triangles[i]);
		Triangulation.HalfEdges.Add(Delaunator.halfedges[i]);
	}
}

//void URegionDistribution::DrawTriangle(UObject* WorldContextObject, int32 Index, const TArray<FVector2D>& OutVerts, const TArray<int32>& Triangles, const TArray<int32>& HalfEdges, float Inset, int32 Label)
void URegionDistribution::DrawTriangle(UObject* WorldContextObject, int32 Index, const FPCGDelaunayTriangulation &Triangulation, float Inset, int32 Label)
{
	if (WorldContextObject)
	{
		auto* World = WorldContextObject->GetWorld();
		if (World)
		{
			int32 TriIndex = Index * 3;

			if (Triangulation.Triangles.IsValidIndex(TriIndex))
			{

				FVector A = FVector(Triangulation.Coords[Triangulation.Triangles[TriIndex]], 0.f);
				FVector B = FVector(Triangulation.Coords[Triangulation.Triangles[TriIndex + 1]], 0.f);
				FVector C = FVector(Triangulation.Coords[Triangulation.Triangles[TriIndex + 2]], 0.f);

				FVector Centre = (A + B + C) / 3.f;
				AManualDetourNavMesh::MoveToCentre(A, Centre, Inset);
				AManualDetourNavMesh::MoveToCentre(B, Centre, Inset);
				AManualDetourNavMesh::MoveToCentre(C, Centre, Inset);

				DrawDebugLine(World, A, B, FColor::White, true, 999.f, 0, Inset * 0.5f);
				DrawDebugLine(World, C, B, FColor::White, true, 999.f, 0, Inset * 0.5f);
				DrawDebugLine(World, A, C, FColor::White, true, 999.f, 0, Inset * 0.5f);

				if (Label >= 0)
				{
					DrawDebugString(World, Centre, FString::FromInt(Label), 0, FColor::White, 999.f);
				}
			}
		}
	}
}

bool URegionDistribution::IsInTriangle(FVector2D P, const FVector2D& A, const FVector2D& B, const FVector2D& C)
{
	if (P == A || P == B || P == C)
		return false;

	FVector UV = GetBaryCentric2D(P, A, B, C);
	return  (UV.X >= 0.f && UV.X <= 1.f &&
			 UV.Y >= 0.f && UV.Y <= 1.f &&
			 UV.Z >= 0.f && UV.Z <= 1.f);
}


FVector URegionDistribution::GetBaryCentric2D(const FVector2D& Point, const FVector2D& A, const FVector2D& B, const FVector2D& C)
{
	float a = ((B.Y-C.Y)*(Point.X-C.X) + (C.X-B.X)*(Point.Y-C.Y)) / ((B.Y-C.Y)*(A.X-C.X) + (C.X-B.X)*(A.Y-C.Y));
	float b = ((C.Y-A.Y)*(Point.X-C.X) + (A.X-C.X)*(Point.Y-C.Y)) / ((B.Y-C.Y)*(A.X-C.X) + (C.X-B.X)*(A.Y-C.Y));

	return FVector(a, b, 1.0f - a - b);	
}

bool URegionDistribution::SegmentIntersectionTest(const FVector2D& SegmentStartA, const FVector2D& SegmentEndA, const FVector2D& SegmentStartB, const FVector2D& SegmentEndB)
{
	const FVector2D VectorA = SegmentEndA - SegmentStartA;
	const FVector2D VectorB = SegmentEndB - SegmentStartB;

	const float S = (-VectorA.Y * (SegmentStartA.X - SegmentStartB.X) + VectorA.X * (SegmentStartA.Y - SegmentStartB.Y)) / (-VectorB.X * VectorA.Y + VectorA.X * VectorB.Y);
	const float T = (VectorB.X * (SegmentStartA.Y - SegmentStartB.Y) - VectorB.Y * (SegmentStartA.X - SegmentStartB.X)) / (-VectorB.X * VectorA.Y + VectorA.X * VectorB.Y);

	return (S >= 0 && S <= 1 && T >= 0 && T <= 1);
}

bool URegionDistribution::BoundsIntersectsSegment(const FBox2D& Bounds, const FVector2D& A, const FVector2D& B)
{
	FVector2D B1 = FVector2D(Bounds.Min.X, Bounds.Max.Y);
	FVector2D B2 = FVector2D(Bounds.Max.X, Bounds.Min.Y);
	return (SegmentIntersectionTest(A, B, Bounds.Min, B1) ||
		SegmentIntersectionTest(A, B, B1, Bounds.Max) ||
		SegmentIntersectionTest(A, B, Bounds.Max, B2) ||
		SegmentIntersectionTest(A, B, B2, Bounds.Min));
}

bool URegionDistribution::IsBoundsCorner(const FBox2D& Bounds, const FVector2D& V)
{
	if (V == Bounds.Min || V == Bounds.Max || V == FVector2D(Bounds.Min.X, Bounds.Max.Y) || V == FVector2D(Bounds.Max.X, Bounds.Min.Y))
	{
		return true;
	}
	return false;
}

int32 URegionDistribution::SegmentInternalColinearityCount(const FVector2D& Q, const FVector2D& R, const FVector2D& A, const FVector2D& B, bool bDirection, bool &bColinear)
{
	int32 Count = 0;
	if (bDirection)
	{
		if (A.X == Q.X && B.X == Q.X)
		{
			if (A.Y > Q.Y && A.Y < R.Y)
			{
				bColinear = true;
				Count++;
			}
			if (B.Y > Q.Y && B.Y < R.Y)
			{
				bColinear = true;
				Count++;
			}
		}
	}
	else {
		if (A.Y == Q.Y && B.Y == Q.Y)
		{
			if (A.X > Q.X && A.X < R.X)
			{
				bColinear = true;
				Count++;
			}
			if (B.X > Q.X && B.X < R.X)
			{
				bColinear = true;
				Count++;
			}
		}
	}
	return Count;
}

int32 URegionDistribution::SegmentColinearWithBoundsPoints(const FBox2D& Bounds, const FVector2D& A, const FVector2D& B, bool &bColinear)
{
	FVector2D B1 = FVector2D(Bounds.Min.X, Bounds.Max.Y);
	FVector2D B2 = FVector2D(Bounds.Max.X, Bounds.Min.Y);

	int32 Count = SegmentInternalColinearityCount(Bounds.Min, B1, A, B, true, bColinear);
	Count += SegmentInternalColinearityCount(Bounds.Min, B2, A, B, false, bColinear);
	Count += SegmentInternalColinearityCount(B1, Bounds.Max, A, B, false, bColinear);
	Count += SegmentInternalColinearityCount(B2, Bounds.Max, A, B, true, bColinear);
	return Count;
}

int32 URegionDistribution::SegmentBoundsIntersections(const FBox2D& Bounds, const FVector2D& A, const FVector2D& B, const FVector2D& C)
{
	FVector2D B1 = FVector2D(Bounds.Min.X, Bounds.Max.Y);
	FVector2D B2 = FVector2D(Bounds.Max.X, Bounds.Min.Y);

	int32 BCCount = 0;
	int32 CACount = 0;

	bool bACorner = IsBoundsCorner(Bounds, A);
	bool bBCorner = IsBoundsCorner(Bounds, B);
	bool bCCorner = IsBoundsCorner(Bounds, C);

	bool bABColinear = false;
	bool bBCColinear = false;
	bool bCAColinear = false;

	// Need to handle the case where each segment could be colinear with the bounds.
	// This should be pretty easy to check, and should return only points that aren't on
	// the corner. Don't treat non-corners as internal since eventually should add which edge
	// is intersected to create tile portals
	// What do do about segments that extend past the corner? Needs to snap to the corner
	//int32 ABCount = SegmentColinearWithBoundsPoints(Bounds, A, B, bABColinear, bABColinear);

	int32 ABCount = 0;
	if (SegmentIntersectionTest(A, B, Bounds.Min, B1))
		ABCount++;
	if(SegmentIntersectionTest(A, B, B1, Bounds.Max))
		ABCount++;
	if (SegmentIntersectionTest(A, B, Bounds.Max, B2))
		ABCount++;
	if (SegmentIntersectionTest(A, B, B2, Bounds.Min))
		ABCount++;

	if (SegmentIntersectionTest(B, C, Bounds.Min, B1))
		BCCount++;
	if(SegmentIntersectionTest(B, C, B1, Bounds.Max))
		BCCount++;
	if (SegmentIntersectionTest(B, C, Bounds.Max, B2))
		BCCount++;
	if (SegmentIntersectionTest(B, C, B2, Bounds.Min))
		BCCount++;

	if (SegmentIntersectionTest(C, A, Bounds.Min, B1))
		CACount++;
	if(SegmentIntersectionTest(C, A, B1, Bounds.Max))
		CACount++;
	if (SegmentIntersectionTest(C, A, Bounds.Max, B2))
		CACount++;
	if (SegmentIntersectionTest(C, A, B2, Bounds.Min))
		CACount++;

	return ABCount + BCCount + CACount;
}
