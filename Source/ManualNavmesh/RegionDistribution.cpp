#include "RegionDistribution.h"
#include "ManualDetourNavMesh.h"
#include "DrawDebugHelpers.h"
#include "Delaunator.h"

bool FPCGDelaunayTriangulation::ValidTriangle(int32 Triangle) const
{
	return
		(Triangles[3 * Triangle] >= 0 &&
			Triangles[3 * Triangle + 1] >= 0 &&
			Triangles[3 * Triangle + 2] >= 0);
}

void FPCGDelaunayTriangulation::RemoveTriangle(int32 Triangle)
{
	Triangles[3*Triangle] = 0;
	Triangles[3*Triangle + 1] = 0;
	Triangles[3*Triangle + 2] = 0;
	HalfEdges[3*Triangle] = -1;
	HalfEdges[3*Triangle + 1] = -1;
	HalfEdges[3*Triangle + 2] = -1;
}

void FPCGDelaunayTriangulation::DeepRemoveTriangle(int32 Triangle)
{
	ReplaceTriangle(Triangle, -1, -1, -1, -1, -1, -1);
}

void FPCGDelaunayTriangulation::ReplaceTriangle(int32 Triangle, int32 P1, int32 P2, int32 P3, int32 E1, int32 E2, int32 E3)
{
	Triangles[3 * Triangle] = P1;
	Triangles[3 * Triangle + 1] = P2;
	Triangles[3 * Triangle + 2] = P3;

	// Remove reciprocal edges
	if (HalfEdges[3 * Triangle] >= 0)
	{
		HalfEdges[HalfEdges[3 * Triangle]] = -1;
	}
	if (HalfEdges[3 * Triangle + 1] >= 0)
	{
		HalfEdges[HalfEdges[3 * Triangle + 1]] = -1;
	}
	if (HalfEdges[3 * Triangle + 2] >= 0)
	{
		HalfEdges[HalfEdges[3 * Triangle + 2]] = -1;
	}

	// Create new edges
	if (E1 < 0)
	{
		HalfEdges[3 * Triangle] = -1;
	}
	else
	{
		HalfEdges[3 * Triangle] = E1;
		HalfEdges[E1] = 3 * Triangle;
	}

	if (E2 < 0)
	{
		HalfEdges[3 * Triangle + 1] = -1;
	}
	else
	{
		HalfEdges[3 * Triangle + 1] = E2;
		HalfEdges[E2] = 3 * Triangle + 1;
	}

	if (E3 < 0)
	{
		HalfEdges[3 * Triangle + 2] = -1;
	}
	else
	{
		HalfEdges[3 * Triangle + 2] = E3;
		HalfEdges[E3] = 3 * Triangle + 2;
	}
}

TriangleNeighbours FPCGDelaunayTriangulation::GetTriangleNeighbours(int32 Triangle)
{
	TriangleNeighbours Ret;
	if (Triangle >= 0)
	{
		for (int32 i = 0; i < 3; i++)
		{
			int32 Edge = TriangleOfEdge(HalfEdges[3 * Triangle + i]);
			if (Edge > 0)
			{
				Ret.Add(Edge);
			}
		}
	}
	return Ret;
}

bool FPCGDelaunayTriangulation::IsInCounterClockwiseDirectionFrom(FVector2D A, FVector2D B)
{
	return A.X * B.Y - A.Y * B.X <= 0.f;
}

bool FPCGDelaunayTriangulation::IsInClockwiseDirectionFrom(FVector2D A, FVector2D B)
{
	return A.X * B.Y - A.Y * B.X >= 0.f;
}

bool FPCGDelaunayTriangulation::PointLiesOnEdge(int32 Edge, FVector2D Point)
{
	if (Edge < 1)
	{
		return false;
	}

	if (Coords[Triangles[Edge]] == Point || Coords[Triangles[NextHalfedge(Edge)]] == Point)
	{
		return true;
	}
	return false;
}

int32 FPCGDelaunayTriangulation::TestPointInTriangle(int32 Triangle, FVector2D Point) const
{
	if (Point == Coords[Triangles[Triangle * 3]])
	{
		return Triangles[Triangle*3];
	}
	if (Point == Coords[Triangles[Triangle * 3+1]])
	{
		return Triangles[Triangle*3+1];
	}
	if (Point == Coords[Triangles[Triangle * 3+2]])
	{
		return Triangles[Triangle*3+2];
	}
	return -1;
}

bool FPCGDelaunayTriangulation::IsInTriangle(FVector2D Point, int32 Triangle, bool bInclusive) const
{
	if (Triangles[3 * Triangle] < 0)
		return false;

	const FVector2D& A = Coords[Triangles[3 * Triangle]];
	const FVector2D& B = Coords[Triangles[3 * Triangle+1]];
	const FVector2D& C = Coords[Triangles[3 * Triangle+2]];
	float u, v, w;

	if (Point == A || Point == B || Point == C)
	{
		return bInclusive;
	}

	if (A == B && B == C)
	{

		UE_LOG(LogManualNavMesh, Warning, TEXT("Tried to find barycentric coods in triangle with area 0"));
		return false;
	}

	FVector2D V0 = B - A;
	FVector2D V1 = C - A;
	FVector2D V2 = Point - A;

	float den = 1 / (V0.X * V1.Y - V1.X * V0.Y);
    v = (V2.X * V1.Y - V1.X * V2.Y) * den;
    w = (V0.X * V2.Y - V2.X * V0.Y) * den;
    u = 1.0f - v - w;

	return !(u < 0.f || u > 1.f || v < 0.f || v > 1.f || w < 0.f || w > 1.f);
}

FVector2D FPCGDelaunayTriangulation::ClosestPointOnEdge(FVector2D Point, int32 Edge)
{
	return FMath::ClosestPointOnSegment2D(Point, Coords[HalfEdgeOrigin(Edge)], Coords[HalfEdgeDest(Edge)]);
}

float FPCGDelaunayTriangulation::DistanceFromEdge(FVector2D Point, int32 Edge)
{
	return FVector2D::Distance(FMath::ClosestPointOnSegment2D(Point, Coords[HalfEdgeOrigin(Edge)], Coords[HalfEdgeDest(Edge)]), Point);
}

bool FPCGDelaunayTriangulation::PointLiesOnEdge(int32 Edge, int32 Point)
{
	if (Edge < 1)
	{
		return false;
	}

	if (Triangles[Edge] == Point || Triangles[NextHalfedge(Edge)] == Point)
	{
		return true;
	}

	return false;
}

int32 FPCGDelaunayTriangulation::GetCommonEdge(int32 Triangle1, int32 Triangle2)
{
	if (Triangle1 < 0)
	{
		return -1;
	}

	if (TriangleOfEdge(HalfEdges[3 * Triangle1]) == Triangle2)
	{
		return 3 * Triangle1;
	}
	if (TriangleOfEdge(HalfEdges[3 * Triangle1 + 1]) == Triangle2)
	{
		return 3 * Triangle1 + 1;
	}
	if (TriangleOfEdge(HalfEdges[3 * Triangle1 + 2]) == Triangle2)
	{
		return 3 * Triangle1 + 2;
	}
	return -1;
}

bool FPCGDelaunayTriangulation::HasEdge(int32 Edge, int32 Triangle2)
{
	if (Triangle2 < 0 || Edge < 0)
	{
		return false;
	}

	if (TriangleOfEdge(Edge) == Triangle2)
	{
		return true;
	}

	if (HalfEdges[3 * Triangle2] == Edge || HalfEdges[3 * Triangle2 + 1] == Edge || HalfEdges[3 * Triangle2 + 2] == Edge)
	{
		return true;
	}

	return false;
}

int32 FPCGDelaunayTriangulation::GetLastValidTriangle(int32 CurrentLast)
{
	int32 Start = CurrentLast >= 0 ? CurrentLast : (Triangles.Num() / 3) - 1;
	for (int32 i = Start; i >= 0; i--)
	{
		if (Triangles[3 * i] >= 0)
		{
			return i;
		}
	}
	// There aren't any valid triangles
	return -1;
}

void FPCGDelaunayTriangulation::SwapTriangles(int32 A, int32 B)
{
	Swap<int32>(Triangles[3 * A], Triangles[3 * B]);
	Swap<int32>(Triangles[3 * A + 1], Triangles[3 * B+1]);
	Swap<int32>(Triangles[3 * A + 2], Triangles[3 * B+2]);

	// Edges can be swapped, but their reciprocals need to be recalculated

	if (HalfEdges[3 * A] >= 0)
	{
		HalfEdges[HalfEdges[3 * A]] = 3 * B;
	}
	if (HalfEdges[3 * A+1] >= 0)
	{
		HalfEdges[HalfEdges[3 * A+1]] = 3 * B+1;
	}
	if (HalfEdges[3 * A+2] >= 0)
	{
		HalfEdges[HalfEdges[3 * A+2]] = 3 * B+2;
	}

	if (HalfEdges[3 * B] >= 0)
	{
		HalfEdges[HalfEdges[3 * B]] = 3 * A;
	}
	if (HalfEdges[3 * B+1] >= 0)
	{
		HalfEdges[HalfEdges[3 * B+1]] = 3 * A+1;
	}
	if (HalfEdges[3 * B+2] >= 0)
	{
		HalfEdges[HalfEdges[3 * B+2]] = 3 * A+2;
	}

	Swap<int32>(HalfEdges[3 * A], HalfEdges[3 * B]);
	Swap<int32>(HalfEdges[3 * A + 1], HalfEdges[3 * B+1]);
	Swap<int32>(HalfEdges[3 * A + 2], HalfEdges[3 * B+2]);
}

void FPCGDelaunayTriangulation::CleanupRemovals()
{
	// Go through the triangulation and find the triangles we nuked by setting the points to -1,
	// then work our way from the *back* of the triangulation and move the last valid triangle into
	// the place with all the -1s.

	// This doesn't remove unused verts

	int32 CurrentValidEnd = GetLastValidTriangle();
	for (int32 i = 0; i < Triangles.Num() / 3; i++)
	{
		if (!ValidTriangle(i))
		{
			SwapTriangles(i, CurrentValidEnd);
			CurrentValidEnd = GetLastValidTriangle(CurrentValidEnd);
		}

		if (i >= CurrentValidEnd)
		{
			break;
		}
	}

	Triangles.SetNum((CurrentValidEnd + 1) * 3);
	HalfEdges.SetNum((CurrentValidEnd + 1) * 3);
}

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

bool URegionDistribution::IsInTriangle(FVector2D P, const FVector2D& A, const FVector2D& B, const FVector2D& C, bool bInclusive)
{
	if (P == A || P == B || P == C)
		return bInclusive;

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
