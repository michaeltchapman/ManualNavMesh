#include "ManualDetourNavMesh.h"
#include "Detour/DetourAlloc.h"
#include "Detour/DetourNavMesh.h"
#include "Detour/DetourNavMeshBuilder.h"
#include "Recast/Recast.h"
#include "Recast/RecastAlloc.h"
#include "NavMesh/PImplRecastNavMesh.h"
#include "NavMesh/RecastHelpers.h"
#include "NavigationSystem/Public/NavigationSystem.h"
#include "NavigationSystem/Public/NavigationSystemTypes.h"
#include "NavigationPath.h"
#include "DrawDebugHelpers.h"
#include "WorldMap.h"
#include "RegionDistribution.h"
#include "RegionPathfinding.h"
#include "ManualNavmesh.h"

DEFINE_LOG_CATEGORY(LogManualNavMesh)

AManualDetourNavMesh::AManualDetourNavMesh()
	: Super()
{
	DebugTile = FIntPoint(-1, -1);
	bDebugDraw = false;
	bNoPortals = false;
	bNoFlags = false;
	TileCount = 2;
	GridSubdivisions = 2;
	TempReservePolys = 256;
	TempReserveVerts = 256;
	EdgeHeightTolerance = 0.01f;
	BaryCentricTolerance = 0.01f;
}

AManualDetourNavMesh* AManualDetourNavMesh::GetManualRecastNavMesh(UObject* Context)
{

	auto* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(Context);
	if (NavSystem)
	{
		for (auto *NavData : NavSystem->NavDataSet)
		{
			AManualDetourNavMesh* ManualNavMesh = Cast<AManualDetourNavMesh>(NavData);
			if (ManualNavMesh)
			{
				return ManualNavMesh;
			}
		}
	}
	return nullptr;
}

FPImplRecastNavMesh *AManualDetourNavMesh::GetPirnm()
{
	return GetRecastNavMeshImpl();
}

FPathFindingResult AManualDetourNavMesh::FindCorridorPathToLocation(FVector Start, FVector Goal, FPathFindingQuery& Query) const
{
	// We could also skip string pulling and get the intersection points while doing that, but we'd need
	// to write our own string pulling to do that
	Query.NavDataFlags = ERecastPathFlags::GenerateCorridor; // | ERecastPathFlags::SkipStringPulling;
	return FindPath(Query.NavAgentProperties, Query);
}

UNavigationPath* AManualDetourNavMesh::FindManualPath(UObject* WorldContextObject, const FVector& PathStart, const FVector& PathEnd, AActor* PathfindingContext,
	TSubclassOf<UNavigationQueryFilter> FilterClass, bool bOptimal, bool bCorridor)
{
	UWorld* World = NULL;

	if (WorldContextObject != NULL)
	{
		World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	}
	if (World == NULL && PathfindingContext != NULL)
	{
		World = GEngine->GetWorldFromContextObject(PathfindingContext, EGetWorldErrorMode::LogAndReturnNull);
	}

	UNavigationPath* ResultPath = NULL;

	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);

	if (NavSys != nullptr && NavSys->GetDefaultNavDataInstance() != nullptr)
	{
		ResultPath = NewObject<UNavigationPath>(NavSys);
		bool bValidPathContext = false;
		AManualDetourNavMesh* NavigationData = NULL;


		if (PathfindingContext != NULL)
		{
			INavAgentInterface* NavAgent = Cast<INavAgentInterface>(PathfindingContext);

			if (NavAgent != NULL)
			{
				const FNavAgentProperties& AgentProps = NavAgent->GetNavAgentPropertiesRef();
				NavigationData = Cast<AManualDetourNavMesh>(NavSys->GetNavDataForProps(AgentProps, PathStart));
				bValidPathContext = true;
			}
			else if (Cast<AManualDetourNavMesh>(PathfindingContext))
			{
				NavigationData = (AManualDetourNavMesh*)PathfindingContext;
				bValidPathContext = true;
			}
		}
		if (bValidPathContext == false)
		{
			// just use default
			NavigationData = Cast<AManualDetourNavMesh>(NavSys->GetDefaultNavDataInstance());
		}

		if (!NavigationData)
		{
			return ResultPath;
		}

		FPathFindingQuery Query(PathfindingContext, *NavigationData, PathStart, PathEnd, UNavigationQueryFilter::GetQueryFilter(*NavigationData, PathfindingContext, FilterClass));

		if (bOptimal)
		{
			/*
			FPathFindingResult Result = NavigationData->FindOptimalPathToLocation(PathStart, PathEnd, Query);
			if (Result.IsSuccessful())
			{
				ResultPath->SetPath(Result.Path);
			}*/
		}

		else if (bCorridor)
		{
			FPathFindingResult Result = NavigationData->FindCorridorPathToLocation(PathStart, PathEnd, Query);
			if (Result.IsSuccessful())
			{
				ResultPath->SetPath(Result.Path);
			}
		}
		else
		{
		}
	}
	return ResultPath;
}

bool AManualDetourNavMesh::FindOptimalPathToLocation(FVector2D Start, FVector2D Goal, UPARAM(ref) TArray<FVector2D> &Path, UPARAM(ref) TArray<FVector> &SurfacePath)
{
#ifdef WITH_EDITOR
	const double timestart = FPlatformTime::Seconds();
#endif // WITH_EDITOR

	// TPAStar to get the path
	if (!FindTPAPath(Start, Goal, Path))
		return false;

	// then use raycasts to get surface path along the height mesh based on the path from TPAStar
	PathToTriangulationSurfacePath(Path, SurfacePath, SourceMap->Triangulation, SourceMap->CoordHeights);

#ifdef WITH_EDITOR
	const double timeend = FPlatformTime::Seconds();
	UE_LOG(LogManualNavMesh, Verbose, TEXT("TPAPath length %d with surface path length %d found in %f seconds."), Path.Num(), SurfacePath.Num(), timeend - timestart);
#endif // WITH_EDITOR

	return true;
}

bool AManualDetourNavMesh::PathToTriangulationSurfacePath(TArray<FVector2D>& Path, UPARAM(ref) TArray<FVector>& SurfacePath,
	UPARAM(ref) FPCGDelaunayTriangulation& Triangulation, UPARAM(ref)  TArray<float>& SurfaceHeights)
{
	if (Path.Num() == 0)
	{
		return false;
	}

	FVector2D Start = Path[0];
	int32 StartTriangle = -1;
	for (int32 i = 0; i < Triangulation.Triangles.Num()/3; i++)
	{
		if (Triangulation.IsInTriangle(Start, i))
		{
			StartTriangle = i;
			break;
		}
	}

	FVector2D End = Path.Last();
	int32 EndTriangle = -1;
	for (int32 i = 0; i < Triangulation.Triangles.Num()/3; i++)
	{
		if (Triangulation.IsInTriangle(End, i))
		{
			EndTriangle = i;
			break;
		}
	}

	if (StartTriangle < 0 || EndTriangle < 0)
	{
		return false;
	}


	int32 PreviousTri = -1;
	int32 CurrentTri = StartTriangle;

	int32 CurrentPathIdx = 0;
	int32 NextPathIdx = 1;

	int32 PointInTriangle = -1;

	FVector VStart = FVector(Start, GetHeightAtPointInTri(Start, Triangulation, StartTriangle, SurfaceHeights));
	SurfacePath.Add(VStart);

	if (Path.Num() == 1)
	{
		return true;
	}
	
	FVector CurrentPathPoint = FVector(Path[CurrentPathIdx], 0.f);
	FVector NextPathPoint =  FVector(Path[NextPathIdx], 0.f);
	FVector IntersectionPoint;

	int32 Catch = 0;

	while (Path.IsValidIndex(NextPathIdx))
	{
		Catch++;
		if (Catch > 100)
		{
			UE_LOG(LogManualNavMesh, Warning, TEXT("TPAPath to Surface projection hit break limit"));
			return false;
		}

		// if the previous placed point was equal to a point on the triangulation then traversing edges naively won't
		// work. Will need to search all triangles connected to the point instead of just neighbours since all of them will
		// match on the wrong edge but only one will match with an unconnected edge
		if (PointInTriangle >= 0)
		{
			// This will be the edge that goes from PointInTriangle to another point
			int32 Point = Triangulation.Triangles[PointInTriangle];

			// This will be the edge that goes from another point to PointInTriangle
			int32 StartEdge = Triangulation.PrevHalfedge(PointInTriangle);

			// Reset this now since the next point might also be a point on the triangulation
			PointInTriangle = -1;

			// Incoming tracks edges that go towards the point we're moving around (CurrentPoint)
			int32 Incoming = StartEdge;
			do {
				int32 Tri = Triangulation.TriangleOfEdge(Incoming);

				// Check if the next point on the path is inside this triangle or intersects with the edge that doesn't contain
				// the previous Point (PointInTriangle)
				if (Triangulation.IsInTriangle(Path[NextPathIdx], Tri))
				{
					UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection added contained point CurrentTri %d, PreviousTri %d, SurfacePath.Num() %d, CurrentPathIdx %d"), CurrentTri, PreviousTri, SurfacePath.Num(), CurrentPathIdx);
					SurfacePath.Add(FVector(Path[NextPathIdx], GetHeightAtPointInTri(Path[NextPathIdx], Triangulation, Tri, SurfaceHeights)));

					if (CurrentTri == EndTriangle)
					{
						return true;
					}

					// The next point we just added might have the same problem
					PointInTriangle = Triangulation.TestPointInTriangle(Tri, Path[NextPathIdx]);
					if (PointInTriangle >= 0)
					{
						UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection added a point %d that was on the corner of triangle %d. Next search will be around a point instead of across an edge"), PointInTriangle, CurrentTri);
					}

					PreviousTri = CurrentTri;
					CurrentTri = Tri;
					CurrentPathIdx++;
					NextPathIdx++;
					CurrentPathPoint = FVector(Path[CurrentPathIdx], 0.f);
					NextPathPoint = FVector(Path[NextPathIdx], 0.f);
					break;
				}

				// Check if the next point on the path intersects with the edge that doesn't contain
				// the previous Point (PointInTriangle)
				int32 OppositeEdge = Triangulation.PrevHalfedge(Incoming);
				int32 Ei1 = Triangulation.Triangles[OppositeEdge];
				int32 Ei2 = Triangulation.Triangles[Incoming];
				FVector Edge1 = FVector(Triangulation.Coords[Ei1], SurfaceHeights[Ei1]);
				FVector Edge2 = FVector(Triangulation.Coords[Ei2], SurfaceHeights[Ei2]);
				if (FMath::SegmentIntersection2D(Edge1, Edge2, CurrentPathPoint, NextPathPoint, IntersectionPoint))
				{
					SurfacePath.Add(IntersectionPoint);
					PreviousTri = CurrentTri;
					CurrentTri = Tri;
					break;
				}
		
				// Move to the next triangle around this point
				int32 Outgoing = Triangulation.NextHalfedge(Incoming);
				Incoming = Triangulation.HalfEdges[Outgoing];
			} while (Incoming != -1 && Incoming != StartEdge);
		}

		// See if the next path point is in this triangle, and increment the path if so
		else if (Triangulation.IsInTriangle(Path[NextPathIdx], CurrentTri))
		{
			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection added contained point CurrentTri %d, PreviousTri %d, SurfacePath.Num() %d, CurrentPathIdx %d"), CurrentTri, PreviousTri, SurfacePath.Num(), CurrentPathIdx);
			SurfacePath.Add(FVector(Path[NextPathIdx], GetHeightAtPointInTri(Path[NextPathIdx], Triangulation, CurrentTri, SurfaceHeights)));

			if (CurrentTri == EndTriangle)
			{
				return true;
			}

			PointInTriangle = Triangulation.TestPointInTriangle(CurrentTri, Path[NextPathIdx]);
			if (PointInTriangle >= 0)
			{
				UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection added a point %d that was on the corner of triangle %d. Next search will be around a point instead of across an edge"), PointInTriangle, CurrentTri);
			}

			CurrentPathIdx++;
			NextPathIdx++;
			CurrentPathPoint = FVector(Path[CurrentPathIdx], 0.f);
			NextPathPoint =  FVector(Path[NextPathIdx], 0.f);

		}
		else {
			// Otherwise, see if the current path segment intersects with either of the edges we didn't enter from
			// and move to the next triangle while adding the segment->edge intersection (using edge height)

			for (int32 i = 0; i < 3; i++)
			{
				int32 TriIdx = CurrentTri * 3;
				int32 Neighbour = Triangulation.TriangleOfEdge(Triangulation.HalfEdges[TriIdx + i]);
				if (Neighbour != PreviousTri)
				{
					int32 Ei1 = Triangulation.Triangles[TriIdx + i];
					int32 Ei2 = Triangulation.Triangles[Triangulation.NextHalfedge(TriIdx + i)];

					FVector Edge1 = FVector(Triangulation.Coords[Ei1], SurfaceHeights[Ei1]);
					FVector Edge2 = FVector(Triangulation.Coords[Ei2], SurfaceHeights[Ei2]);

					// Have the height segment first so the resulting point will have a height
					if (FMath::SegmentIntersection2D(Edge1, Edge2, CurrentPathPoint, NextPathPoint, IntersectionPoint))
					{
						if (FVector2D(IntersectionPoint) != Triangulation.Coords[Ei1] && FVector2D(IntersectionPoint) != Triangulation.Coords[Ei2])
						{
							UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection added intersection point CurrentTri %d, PreviousTri %d, SurfacePath.Num() %d, CurrentPathIdx %d"), CurrentTri, PreviousTri, SurfacePath.Num(), CurrentPathIdx);
							SurfacePath.Add(IntersectionPoint);
							PreviousTri = CurrentTri;
							CurrentTri = Neighbour;
							break;
						}
						else {
							UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection failed corner check"));
						}
					}
					else {
						UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection failed intersection check"));
					}
				}
			}

			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("TPAPath to Surface projection fell through"));
		}

	}

	return true;
}

bool AManualDetourNavMesh::PathToSurfacePath(UNavigationPath* Path, UPARAM(ref)  TArray<FVector>& SurfacePath)
{
	if (!Path || Path->PathPoints.Num() == 0 || !Path->GetPath())
	{
		return false;
	}

	FNavPathSharedPtr SharedPath = Path->GetPath();
	FNavMeshPath *NavPath = Path->GetPath().Get()->CastPath<FNavMeshPath>();

	//FNavigationPath* NavPath = Query.PathInstanceToFill.Get();
	//FNavMeshPath* NavMeshPath = NavPath ? NavPath->CastPath<FNavMeshPath>() : nullptr;

	if (NavPath)
	{
		return PathCorridorToSurfacePath(NavPath->GetPathPoints(), NavPath->GetPathCorridorEdges(), SurfacePath);
	}
	else {
		return false;
	}
}

bool AManualDetourNavMesh::PathCorridorToSurfacePath(UPARAM(ref) const TArray<struct FNavPathPoint>& Path, UPARAM(ref) const TArray<FNavigationPortalEdge>& Corridor,UPARAM(ref)  TArray<FVector>& SurfacePath)
{
	int32 CurrentPathIdx = 0;
	int32 NextPathIdx = 1;
	int32 CurrentEdgeIdx = 0;

	if (Path.Num() == 0 || Corridor.Num() == 0)
	{
		return false;
	}

	SurfacePath.Reset(Path.Num() + Corridor.Num());
	SurfacePath.Add(Path[0].Location);

	while (Path.IsValidIndex(NextPathIdx))
	{
		// If there are still corridor edges to check, try that
		if (Corridor.IsValidIndex(CurrentEdgeIdx))
		{
			// If current to next intersects current edge, add intersection point
			FVector IntersectionPoint;
			if (FMath::SegmentIntersection2D(Corridor[CurrentEdgeIdx].Left, Corridor[CurrentEdgeIdx].Right, Path[CurrentPathIdx].Location, Path[NextPathIdx].Location, IntersectionPoint))
			{
				//float Ratio = GetRatioFromTriSegment(IntersectionPoint, Corridor[CurrentEdgeIdx].Left, Corridor[CurrentEdgeIdx].Right);
				//IntersectionPoint.Z = Corridor[CurrentEdgeIdx].Right.Z* Ratio + Corridor[CurrentEdgeIdx].Left.Z * (1.f - Ratio);
				SurfacePath.Add(IntersectionPoint);
				CurrentEdgeIdx++;
			}
			// otherwise add the next point and move forward
			else {
				SurfacePath.Add(Path[NextPathIdx].Location);
				CurrentPathIdx++;
				NextPathIdx++;
			}
		}
		// If we've checked everything in the corridor, just add next point and move forward
		else {
			SurfacePath.Add(Path[NextPathIdx].Location);
			CurrentPathIdx++;
			NextPathIdx++;
		}
	}

	return (SurfacePath.Num() == Path.Num() + Corridor.Num());
}

bool AManualDetourNavMesh::FindTPAPath(FVector2D Start, FVector2D End, UPARAM(ref) TArray<FVector2D> &OutPath) const
{
	if (!SourceMap)
		return false;

	FPCGDelaunayTriangulation& Triangulation = SourceMap->GetTPATriangulation();
	int32 StartTriangle = -1;
	for (int32 i = 0; i < Triangulation.Triangles.Num()/3; i++)
	{
		if (Triangulation.IsInTriangle(Start, i))
		{
			StartTriangle = i;
			break;
		}
	}

	if (StartTriangle < 0)
		return false;

	FPCGTPAPathFinder PathFinder;// = FPCGTPAPathFinder(&SourceMap->Triangulation);
	PathFinder.SetTriangulation(&Triangulation);
	PathFinder.FindPath(Start, StartTriangle, End, OutPath);

	return true;
}

void AManualDetourNavMesh::MakeInitParams(dtNavMeshParams& inParams, const float* bmin, float inTileSize)
{
	memset(&inParams, 0, sizeof(inParams));
	inParams.orig[0] = bmin[0];
	inParams.orig[1] = bmin[1];
	inParams.orig[2] = bmin[2];
	inParams.tileWidth = inTileSize;
	inParams.tileHeight = inTileSize;
	inParams.maxTiles = 256; // TODO this should be exact
	inParams.maxPolys = 0xffff; // TODO this should be exact
}

bool AManualDetourNavMesh::BuildManualNavMesh()
{
#if WITH_EDITOR
	const double timestart = FPlatformTime::Seconds();
#endif WITH_EDITOR

	FPImplRecastNavMesh *Pirnm = GetPirnm();
	if (!Pirnm)
	{
		return false;
	}

	if (!SourceMap)
	{
		UE_LOG(LogManualNavMesh, Warning, TEXT("Cannot initialise manual nav mesh without a source WorldMap"));
		return false;
	}
	Max = SourceMap->GetMax();
	Min = SourceMap->GetMin();

	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Initialising Nav Mesh Tile with Min %s, Max %s"), *Min.ToString(), *Max.ToString());
	const float bmax[3] =
	{
		Max.X,
		Max.Z,
		Max.Y
	};

	const float bmin[3] =
	{
		Min.X,
		Min.Z,
		Min.Y
	};

	FPImplRecastNavMesh* PIRNM = GetPirnm();

	if (PIRNM->DetourNavMesh != nullptr)
	{
		dtFreeNavMesh(PIRNM->DetourNavMesh);
	}
	// Allocates memory but doesn't initialise
	PIRNM->DetourNavMesh = dtAllocNavMesh();

	/*
		normally a fixed cell size is used as a param, but since we know the total
		bounds of the mesh being passed in, and we know the total number of cells
		is SHORT_MAX (32767) we can probably do better

		eg. For a 2000x2000 map each cell will be 0.06

		?? is this valuable? Maybe just use 1cm unless there's some actual need for mm-accurate
		navigation...
	*/

	float MaxSize = FMath::Max(Max.X - Min.X, Max.Y - Min.Y);
	if (CellSize <= 0.f)
	{
		CellSize = MaxSize / 32767.f;
	}

	if (TileCount <= 0)
	{
		UE_LOG(LogManualNavMesh, Error, TEXT("TileCount value of %d is invalid for nav mesh generation"), TileCount);
		return false;
	}

	TileSize = (Max.X - Min.X) / TileCount;

	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Initialising Nav Mesh Params with TileSize %s"), *FString::SanitizeFloat(TileSize));

	dtNavMeshParams NMParams;
	MakeInitParams(NMParams, bmin, TileSize);

	dtStatus status = PIRNM->DetourNavMesh->init(&NMParams);

	for (int y = 0; y < TileCount; y++)
	{
		for (int x = 0; x < TileCount; x++)
		{
			if (bDebugDraw && DebugTile.X >= 0 && DebugTile.Y >= 0 && (x != DebugTile.X || y != DebugTile.Y))
			{
				UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("skipping tile %d,%d"), x, y);
				continue;
			}

			float TileBmin[3] = {
				bmin[0] + x * TileSize,
				bmin[1],
				bmin[2] + y * TileSize,
			};

			float TileBmax[3] = {
				bmin[0] + (x + 1) * TileSize,
				bmax[1],
				bmin[2] + (y + 1) * TileSize,
			};

			int dataSize = 0;

			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Creating tile %d,%d. Min: %s Max: %s"), x, y,
				*FVector2D(TileBmin[0], TileBmin[2]).ToString(),
				*FVector2D(TileBmax[0], TileBmax[2]).ToString());

			unsigned char* data = BuildTileMesh(PIRNM->DetourNavMesh, x, y, TileBmin, TileBmax, dataSize);
			if (data)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				PIRNM->DetourNavMesh->removeTile(PIRNM->DetourNavMesh->getTileRefAt(x, y, 0), 0, 0);
				// Let the navmesh own the data.
				status = PIRNM->DetourNavMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);
				UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Tile %d,%d creation status %s"), x,y,*DetourStatusToString(status))
				if (dtStatusFailed(status))
				{
					UE_LOG(LogManualNavMesh, Error, TEXT("Tile %d,%d creation failed with status %s"), x,y,*DetourStatusToString(status))
					dtFree(data);
				}
			}
		}
	}

	UpdateDrawing();

#ifdef WITH_EDITOR
	const double timeend = FPlatformTime::Seconds();
	UE_LOG(LogManualNavMesh, Verbose, TEXT("Manual Nav Mesh built in %f seconds."), timeend - timestart);

#endif // WITH_EDITOR

	return true;

}

bool AManualDetourNavMesh::SetNavMeshFromVerts(TArray<FVector>& Vertices, TArray<int32>& Indices)
{
	return false;
}


FString AManualDetourNavMesh::AManualDetourNavMesh::DetourStatusToString(unsigned int status)
{
	FString Ret = FString();
	if ((status & DT_SUCCESS) != 0)
		Ret.Append(FString(TEXT(" DT_SUCCESS")));
	if ((status & DT_FAILURE) != 0)
		Ret.Append(FString(TEXT(" DT_FAILURE")));
	if ((status & DT_IN_PROGRESS) != 0)
		Ret.Append(FString(TEXT(" DT_IN_PROGRESS")));
	if ((status & DT_STATUS_DETAIL_MASK) != 0)
		Ret.Append(FString(TEXT(" DT_STATUS_DETAIL_MASK")));
	if ((status & DT_WRONG_MAGIC) != 0)
		Ret.Append(FString(TEXT(" DT_WRONG_MAGIC")));
	if ((status & DT_WRONG_VERSION) != 0)
		Ret.Append(FString(TEXT(" DT_WRONG_VERSION")));
	if ((status & DT_OUT_OF_MEMORY) != 0)
		Ret.Append(FString(TEXT(" DT_OUT_OF_MEMORY")));
	if ((status & DT_INVALID_PARAM) != 0)
		Ret.Append(FString(TEXT(" DT_INVALID_PARAM")));
	if ((status & DT_BUFFER_TOO_SMALL) != 0)
		Ret.Append(FString(TEXT(" DT_BUFFER_TOO_SMALL")));
	if ((status & DT_OUT_OF_NODES) != 0)
		Ret.Append(FString(TEXT(" DT_OUT_OF_NODES")));
	if ((status & DT_PARTIAL_RESULT) != 0)
		Ret.Append(FString(TEXT(" DT_PARTIAL_RESULT")));
	if ((status & DT_INVALID_CYCLE_PATH) != 0)
		Ret.Append(FString(TEXT(" DT_INVALID_CYCLE_PATH")));
	return Ret;
}

unsigned short AManualDetourNavMesh::DirectionToSide(EPCGDirection Border)
{

	switch (Border)
	{
	case EPCGDirection::E_North:
		return 0x8000 | 3;
		break;
	case EPCGDirection::E_South:
		return 0x8000 | 1;
		break;
	case EPCGDirection::E_East:
		return 0x8000 | 0;
		break;
	case EPCGDirection::E_West:
		return 0x8000 | 2;
		break;
	case EPCGDirection::E_None:
		return 0xffff;
		break;
	default:
		break;
	}
	return 0xffff;
}

unsigned char* AManualDetourNavMesh::BuildTileMesh(dtNavMesh* TiledNavMesh, const int32 tx, const int32 ty, const float* bmin, const float* bmax, int32 &NavDataSize)
{

	// Bounds is used for checking polys, rather than verts, so doesn't
	// need the border
	FBox2D Bounds = FBox2D(
		FVector2D(-1.f * bmax[0], -1.f * bmax[2]),
		FVector2D(-1.f * bmin[0], -1.f * bmin[2])
	);

	if (Bounds.GetExtent().ContainsNaN() || Bounds.GetCenter().ContainsNaN())
	{
		UE_LOG(LogManualNavMesh, Error, TEXT("Tile Bounds Contain NaN: %s"), *Bounds.ToString());
		return nullptr;
	}

	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Tile Bounds: %s"), *Bounds.ToString());

	dtNavMeshCreateParams TileParams;
	FMemory::Memset(&TileParams, 0, sizeof(TileParams));
	unsigned char* navData = 0;
	int navDataSize = 0;

	TileParams.walkableHeight = WalkableHeight;
	TileParams.walkableRadius = WalkableRadius;
	TileParams.walkableClimb = WalkableClimb;
	TileParams.tileX = tx;
	TileParams.tileY = ty;
	TileParams.tileLayer = 0;
	rcVcopy(TileParams.bmin, bmin);
	rcVcopy(TileParams.bmax, bmax);
	TileParams.cs = CellSize;
	TileParams.ch = CellHeight;
	TileParams.buildBvTree = true;

	if (!SourceMap)
	{
		return nullptr;
	}

	if (SourceMap->IsGrid())
	{
		MakeGridTile(TileParams, tx, ty);
	}
	else {
		//MakeTriangulationTile(TileParams, tx, ty, SourceMap->Coords, SourceMap->Triangles, SourceMap->HalfEdges);
		MakeTriangulationTile(TileParams, tx, ty, SourceMap->Triangulation);
	}

	if (TileParams.vertCount < 3)
	{
		UE_LOG(LogManualNavMesh, Verbose, TEXT("Tile %d,%d vertcount %d too low to create tile"), tx, ty, TileParams.vertCount);
		return nullptr;
	}
	if (TileParams.vertCount >= 0xffff)
	{
		GLog->Log(TEXT("Too many verts in navmesh tile"));
		UE_LOG(LogManualNavMesh, Error, TEXT("Tile %d,%d vertcount %d too high"), tx, ty, TileParams.vertCount);
		return nullptr;
	}

	if (bDebugDraw && UE_LOG_ACTIVE(LogManualNavMesh, Verbose))
	{
		DebugDrawTileParams(TileParams);
	}

	// Make the nav mesh
	if (!dtCreateNavMeshData(&TileParams, &navData, &navDataSize))
	{
		GLog->Log(TEXT("Could not build Detour navmesh."));
		return 0;
	}
	NavDataSize = navDataSize;
	return navData;
}

void AManualDetourNavMesh::MakeGridTile(dtNavMeshCreateParams& TileParams, const int32 tx, const int32 ty)
{

	// Bounds is used for checking polys, rather than verts, so doesn't
	// need the border
	FBox2D Bounds = FBox2D(
		FVector2D(-1.f * TileParams.bmax[0], -1.f * TileParams.bmax[2]),
		FVector2D(-1.f * TileParams.bmin[0], -1.f * TileParams.bmin[2])
	);

	GridSubdivisions = GridSubdivisions <= 0 ? 1 : GridSubdivisions;

	// We want [GridSubdivisions] quads on each row so we need one more vertex at the end
	unsigned short RowLength = GridSubdivisions + 1;
	TileParams.vertCount = RowLength * RowLength;

	// Each poly will be a quad
	int MaxPolys = GridSubdivisions * GridSubdivisions;

	unsigned short CellsInTile = FMath::FloorToInt((Bounds.Max.X - Bounds.Min.X) / TileParams.cs);
	unsigned short CellsInSubdiv = CellsInTile / GridSubdivisions;

	// Vertices per polygon
	TileParams.nvp = 4;

	// These will be the same apart from fixed interval vs floating point.
	unsigned short* verts = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * TileParams.vertCount * 3);

	for (int32 j = 0; j < RowLength; j++)
	{
		for (int32 i = 0; i < RowLength; i++)
		{
			float X = Bounds.Min.X + i * CellsInSubdiv * TileParams.cs;
			float Y = Bounds.Min.Y + j * CellsInSubdiv * TileParams.cs;
			SetDetourVector(verts, FVector(X, Y, GetHeightAt(X,Y)), j * RowLength + i, TileParams);
			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("GetHeightAt %s"), *FString::SanitizeFloat(GetHeightAt(X, Y)));
		}
	}

	// This is list of vert indices with additional edge flags. 0xFF means no vert index and solid border edge
	unsigned short* polys = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * MaxPolys * TileParams.nvp * 2);
	memset(polys, 0xff, sizeof(unsigned short) * MaxPolys * TileParams.nvp * 2);

	for (int32 j = 0; j < GridSubdivisions; j++)
	{
		for (int32 i = 0; i < GridSubdivisions; i++)
		{
			unsigned short PolyIndex = (i + j * GridSubdivisions)*TileParams.nvp*2;

			polys[PolyIndex]     = j * RowLength + i;
			polys[PolyIndex + 1] = (j + 1) * RowLength + i;
			polys[PolyIndex + 2] = (j + 1) * RowLength + i+1;
			polys[PolyIndex + 3] = j * RowLength + i+1;

			if (i == GridSubdivisions - 1)
			{
				if (TileParams.tileX == 0)
				{
					// Mesh Border
					polys[PolyIndex + 6] = RC_MESH_NULL_IDX;
				}
				else {
					// Tile Border to another tile
					polys[PolyIndex + 6] = 0x8000;
				}
			}
			else {
				// Subdiv border to another poly
				polys[PolyIndex + 6] = (i+1) + j*GridSubdivisions;
			}

			if (j == 0)
			{
				if (TileParams.tileY == TileCount - 1)
				{
					polys[PolyIndex + 7] = RC_MESH_NULL_IDX;
				}
				else {
					polys[PolyIndex + 7] = 0x8001;
				}
			}
			else {
				polys[PolyIndex + 7] = i + (j-1)*GridSubdivisions;
			}

			if (i == 0)
			{
				if (TileParams.tileX == TileCount - 1)
				{
					polys[PolyIndex + 4] = RC_MESH_NULL_IDX;
				}
				else {
					polys[PolyIndex + 4] = 0x8002;
				}
			}
			else {
				// Subdiv border to another poly
				polys[PolyIndex + 4] = i-1 + j*GridSubdivisions;
			}

			if (j == GridSubdivisions - 1)
			{
				if (TileParams.tileY == 0)
				{
					polys[PolyIndex + 5] = RC_MESH_NULL_IDX;
				}
				else {
					polys[PolyIndex + 5] = 0x8003;
				}
			}
			else {
				// Subdiv border to another poly
				polys[PolyIndex + 5] = i + (j+1)*GridSubdivisions;
			}
		}
	}

	// This is area IDs, not the area of the polygon.
	unsigned char* PolyAreas = (unsigned char*)FMemory::Malloc(sizeof(unsigned char) * MaxPolys);
	//FMemory::Memset(PolyAreas, 0xff, sizeof(unsigned char)*MaxPolys);
	FMemory::Memset(PolyAreas, RC_WALKABLE_AREA, sizeof(unsigned char) * MaxPolys);

	// 0xff seems to mean walkable. 0x00 seems to mean impassable. Recast seems to generate 0x0001 but not sure
	// what that means
	unsigned short* PolyFlags = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * MaxPolys);
	//FMemory::Memset(PolyFlags, 0xff, sizeof(unsigned short) * MaxPolys);
	if (bNoFlags)
	{
		FMemory::Memset(PolyFlags, 0xff, sizeof(unsigned short) * MaxPolys);
	}
	else {
		FMemory::Memset(PolyFlags, 0x0001, sizeof(unsigned short) * MaxPolys);
	}
	TileParams.polyFlags = PolyFlags;

	// Off mesh connections
	TileParams.offMeshConCount = 0;
	TileParams.offMeshCons = 0;

	// Will be created by the init method based on the base poly
	TileParams.detailMeshes = 0;
	TileParams.detailVerts = 0;
	TileParams.detailVertsCount = 0;
	TileParams.detailTris = 0;
	TileParams.detailTriCount = 0;

	TileParams.verts = verts;
	TileParams.polys = polys;
	TileParams.polyAreas = PolyAreas;
	TileParams.polyFlags = PolyFlags;
	TileParams.polyCount = MaxPolys;
}

void AManualDetourNavMesh::DebugDrawTileParams(dtNavMeshCreateParams& TileParams)
{
	static FColor VertexColor = FColor::Purple;

	for (unsigned short i = 0; i < TileParams.polyCount; i++)// *TileParams.nvp * 2; i += TileParams.nvp * 2)
	{
		int32 PolyStart = i * TileParams.nvp * 2;
		FVector PolyCentre = GetPolyCentre(TileParams, i);

		// Poly Label
		DrawDebugString(GetWorld(), PolyCentre, *FString::FromInt(i), 0, FColor::Orange, 999.f);

		for (int32 j = 0; j < TileParams.nvp; j++)
		{
			// First vert index for the edge
			unsigned short Ind1 = TileParams.polys[PolyStart+j];

			// If current vert is null we can stop
			if (Ind1 == RC_MESH_NULL_IDX)
			{
				break;
			}

			// Second vert index for the edge, wrap back around at both the end of the poly and
			// if the next index is null.
			unsigned short Ind2 = (j == TileParams.nvp - 1 || TileParams.polys[PolyStart+j+1] == RC_MESH_NULL_IDX) ? TileParams.polys[PolyStart] : TileParams.polys[PolyStart+j+1];

			FVector V1 = GetVector(TileParams, Ind1);
			FVector V2 = GetVector(TileParams, Ind2);

			unsigned short EdgeCode = TileParams.polys[PolyStart + TileParams.nvp + j];
			FColor EdgeColor = GetEdgeDebugColor(EdgeCode);

			// Edge label
			if (EdgeColor == FColor::Silver)
			{
				FVector E = (V1 + V2) * 0.5f;
				MoveToCentre(E, PolyCentre, DebugThickness);
				DrawDebugString(GetWorld(), E, *FString::FromInt(EdgeCode), 0, FColor::Green, 999.f);
			}

			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge %d of poly %2d EdgeCode %4hu (%2hu)-(%2hu) :: %s - %s"), j, i, EdgeCode, Ind1, Ind2, *V1.ToString(), *V2.ToString());

			MoveToCentre(V1, PolyCentre,DebugThickness);
			MoveToCentre(V2, PolyCentre,DebugThickness);

			DrawDebugLine(GetWorld(), V1, V2, EdgeColor, true, 999.f, 0, DebugThickness);

			MoveToCentre(V1, PolyCentre,DebugThickness);

			// Vertex label
			DrawDebugString(GetWorld(), V1, *FString::FromInt(Ind1), 0, VertexColor, 999.f);

		}
	}
}

FVector AManualDetourNavMesh::GetVector(dtNavMeshCreateParams& TileParams, unsigned short Index)
{
	if (Index >= TileParams.vertCount)
	{
		UE_LOG(LogManualNavMesh, Warning, TEXT("Requested vector %d from detour but only %d vectors available"), Index, TileParams.vertCount);
		return FVector::ZeroVector;
	}

	return FVector(
		TileParams.verts[3 * Index] * TileParams.cs * -1.f - TileParams.bmin[0],
		TileParams.verts[3 * Index+2] * TileParams.cs * -1.f - TileParams.bmin[2],
		TileParams.verts[3 * Index+1] * TileParams.ch + TileParams.bmin[1]
	);
}

void AManualDetourNavMesh::SetDetourVector(unsigned short* verts, const FVector V, unsigned short Index, dtNavMeshCreateParams& TileParams)
{
	const float ncsinv = -1.0f / TileParams.cs;
	const float chinv = 1.0f / TileParams.ch;

	int32 x = FMath::FloorToInt((TileParams.bmin[0] + V.X) * ncsinv);
	int32 z = FMath::FloorToInt((V.Z - TileParams.bmin[1]) * chinv);
	int32 y = FMath::FloorToInt((TileParams.bmin[2] + V.Y) * ncsinv);

	verts[Index * 3] = (unsigned short)(x >= 0 ? x : 0);
	verts[Index * 3 + 1] = (unsigned short)(z >= 0 ? z : 0);
	verts[Index * 3 + 2] = (unsigned short)(y >= 0 ? y : 0);

	// Correct anything that underflowed. Overflow is really unlikely so we can probably ignore it
	/*
	verts[Index * 3] = verts[Index * 3] == 0xffff ? 0 : verts[Index * 3];
	verts[Index * 3+1] = verts[Index * 3+1] == 0xffff ? 0 : verts[Index * 3+1];
	verts[Index * 3+2] = verts[Index * 3+2] == 0xffff ? 0 : verts[Index * 3+2];*/
}

FVector AManualDetourNavMesh::GetPolyCentre(dtNavMeshCreateParams& TileParams, unsigned short Index)
{
	if (Index >= TileParams.polyCount)
	{
		UE_LOG(LogManualNavMesh, Warning, TEXT("Requested centre of poly %d from detour but only %d polys available"), Index, TileParams.polyCount);
		return FVector::ZeroVector;
	}

	int32 Count = 0;
	FVector Ret = FVector::ZeroVector;
	int32 Start = Index * TileParams.nvp * 2;
	for (int32 i = Start; i < Start + TileParams.nvp; i++)
	{
		if (TileParams.polys[i] == RC_MESH_NULL_IDX)
			break;
		Ret += GetVector(TileParams, TileParams.polys[i]);
		Count++;
	}

	Ret /= (float)Count;
	return Ret;
}

EPCGTriPoint AManualDetourNavMesh::GetCommonEdge(EPCGTriPoint A, EPCGTriPoint B)
{
	if ((A == EPCGTriPoint::E_East || A == EPCGTriPoint::E_NorthEast || A == EPCGTriPoint::E_SouthEast) &&
		(B == EPCGTriPoint::E_East || B == EPCGTriPoint::E_NorthEast || B == EPCGTriPoint::E_SouthEast))
	{
		return EPCGTriPoint::E_East;
	}

	if ((A == EPCGTriPoint::E_West || A == EPCGTriPoint::E_NorthWest || A == EPCGTriPoint::E_SouthWest) &&
		(B == EPCGTriPoint::E_West || B == EPCGTriPoint::E_NorthWest || B == EPCGTriPoint::E_SouthWest))
	{
		return EPCGTriPoint::E_West;
	}

	if ((A == EPCGTriPoint::E_South || A == EPCGTriPoint::E_SouthEast || A == EPCGTriPoint::E_SouthWest) &&
		(B == EPCGTriPoint::E_South || B == EPCGTriPoint::E_SouthEast || B == EPCGTriPoint::E_SouthWest))
	{
		return EPCGTriPoint::E_South;
	}

	if ((A == EPCGTriPoint::E_North || A == EPCGTriPoint::E_NorthEast || A == EPCGTriPoint::E_NorthWest) &&
		(B == EPCGTriPoint::E_North || B == EPCGTriPoint::E_NorthEast || B == EPCGTriPoint::E_NorthWest))
	{
		return EPCGTriPoint::E_North;
	}
	return EPCGTriPoint::E_External;
}

unsigned short AManualDetourNavMesh::GetEdgePortal(EPCGTriPoint Edge)
{
	switch (Edge)
	{
	case EPCGTriPoint::E_West:
		return 0x8002;
	case EPCGTriPoint::E_North:
		return 0x8003;
	case EPCGTriPoint::E_East:
		return 0x8000;
	case EPCGTriPoint::E_South:
		return 0x8001;
	default:
		return RC_MESH_NULL_IDX;
	}
}

FColor AManualDetourNavMesh::GetEdgeDebugColor(unsigned short EdgeCode)
{
	static FColor BorderColor = FColor::Cyan;
	static FColor InternalEdgeColor = FColor::Silver;
	static FColor WestPortalColor = FColor::Red;
	static FColor EastPortalColor = FColor::Green;
	static FColor SouthPortalColor = FColor::Blue;
	static FColor NorthPortalColor = FColor::Yellow;

	switch (EdgeCode)
	{
	case 0x8000:
		return WestPortalColor;
	case 0x8001:
		return NorthPortalColor;
	case 0x8002:
		return EastPortalColor;
	case 0x8003:
		return SouthPortalColor;
	case RC_MESH_NULL_IDX:
		return BorderColor;
	default:
		return InternalEdgeColor;
	}

}

bool AManualDetourNavMesh::IsEdge(EPCGTriMember Member)
{
	return (Member == EPCGTriMember::E_AB || Member == EPCGTriMember::E_CA || Member == EPCGTriMember::E_BC ||
	        Member == EPCGTriMember::E_BA || Member == EPCGTriMember::E_AC || Member == EPCGTriMember::E_CB);
}

bool AManualDetourNavMesh::IsPoint(EPCGTriMember Member)
{
	return (Member == EPCGTriMember::E_A || Member == EPCGTriMember::E_C || Member == EPCGTriMember::E_C);
}

void AManualDetourNavMesh::MoveToCentre(FVector& InVector, const FVector Centre, float Distance)
{
	FVector Dir = (Centre - InVector).GetSafeNormal();
	InVector += Dir * Distance*2.f;
}

float AManualDetourNavMesh::GetHeightAt(float X, float Y)
{
	if (SourceMap)
	{
		return Min.Z + (Max.Z - Min.Z) * SourceMap->GetHeightAt(X, Y);
	}
	return 0.f;
}

int32 AManualDetourNavMesh::GetTileCount() const
{
	return TileCount;
}

int32 AManualDetourNavMesh::GetGridSubdivisions() const
{
	return GridSubdivisions;
}

void AManualDetourNavMesh::MakeTriangulationTile(dtNavMeshCreateParams& TileParams, const int32 tx, const int32 ty, const FPCGDelaunayTriangulation &Triangulation)
{
	FBox2D Bounds = FBox2D(
		FVector2D(-1.f * TileParams.bmax[0], -1.f * TileParams.bmax[2]),
		FVector2D(-1.f * TileParams.bmin[0], -1.f * TileParams.bmin[2])
	);

	unsigned short CellsInTile = FMath::FloorToInt((Bounds.Max.X - Bounds.Min.X) / TileParams.cs);

	/*
#if WITH_EDITOR
	if (bDebugDraw)
	{
		DrawDebugBox(GetWorld(), FVector(Bounds.GetCenter(), 0.f), FVector(Bounds.GetExtent(), 1.f), FColor::Cyan, true, 999.f, 0, DebugThickness);
	}
#endif*/

	//TArray<FVector2D> TmpVerts;
	TArray<FVector> TmpVerts;
	TmpVerts.Reserve(TempReserveVerts);

	TArray<unsigned short> TmpPolys;
	TmpPolys.AddDefaulted(4 * SourceMap->Triangulation.Triangles.Num());
	FMemory::Memset(TmpPolys.GetData(), 0xff, sizeof(unsigned short) * 4 * SourceMap->Triangulation.Triangles.Num());


	// Mapping from triangulation vert index to vert in this tile
	TMap<int32, FPCGTriPoint> InternalVerts;
	InternalVerts.Reserve(TempReserveVerts);

	// Mapping from triangulation half edge index to a derived vert in this tile
	TMap<int32, FPCGTriPoint> HalfEdgeVerts;
	HalfEdgeVerts.Reserve(TempReserveVerts);

	// Mapping from triangule in triangulation to poly in tile
	TMap<int32, int32> PolyMap;
	PolyMap.Reserve(TempReservePolys);

	// The max we can have is 6. This might be wasteful for some tiles but it shouldn't make a huge difference
	TileParams.nvp = 6;

	int32 TmpCurrentPoly = 0;
	for (int32 i = 0; i < Triangulation.Triangles.Num() / 3; i++)
	{
		if (Triangulation.ValidTriangle(i))
		{
			int32 VertCount = ProcessTriangle(Bounds, i, Triangulation, SourceMap->CoordHeights, TmpVerts, TmpPolys, TmpCurrentPoly, InternalVerts, HalfEdgeVerts);
			if (VertCount > 0)
			{
				UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Triangle %d has %d verts"), i, VertCount);
				PolyMap.Add(i, TmpCurrentPoly);

				URegionDistribution::DrawTriangle(this, i, Triangulation, 1.f, i);

				// Swap winding order ?
				//ReversePolyVertOrder(TmpCurrentPoly, VertCount, TmpPolys);

				TmpCurrentPoly++;
			}
		}
	}
//	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("MaxPolyVerts for tile %d, Corner Verts %d, EdgeVerts, %d, InternalVerts %d"), MaxPolyVerts, CornerVerts, EdgeVerts, InternalVerts.Num());
	// Vertices per polygon
	TileParams.vertCount = TmpVerts.Num();
	TileParams.polyCount = PolyMap.Num();

	/*
#if WITH_EDITOR
	if (bDebugDraw)
	{
		for (auto Tri : PolyMap)
		{
			URegionDistribution::DrawTriangle(this, Tri.Value, Triangulation, DebugThickness, Tri.Value);
		}
	}
#endif WITH_EDITOR
*/

	unsigned short* verts = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * TileParams.vertCount * 3);
	unsigned short* polys = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * TileParams.polyCount * TileParams.nvp * 2);

	FMemory::Memcpy(polys, TmpPolys.GetData(), sizeof(unsigned short) * TileParams.polyCount * TileParams.nvp * 2);

	for (int32 i = 0; i < TmpVerts.Num(); i++)
	{
		//SetDetourVector(verts, FVector(TmpVerts[i].X, TmpVerts[i].Y, 0.f), i, TileParams);
		SetDetourVector(verts, TmpVerts[i], i, TileParams);
	}

	// TODO memcpy tmppolys to polys
	for (int32 i = 0; i < PolyMap.Num(); i++)
	{
		int32 PolyIdx = i * TileParams.nvp * 2;
		int32 AdjacencyIdx = PolyIdx + TileParams.nvp;
		for (int32 j = AdjacencyIdx; j < AdjacencyIdx + TileParams.nvp; j++)
		{
			if (!IsBorder(TmpPolys[j]))
			{
				int32* MappedPoly = PolyMap.Find((int32)TmpPolys[j]);
				if (MappedPoly)
				{
					polys[j] = (unsigned short)*MappedPoly;

				}
				else {
					UE_LOG(LogManualNavMesh, Warning, TEXT("Poly %d not found in poly map for index %d, poly %d"), TmpPolys[j], j, i);
				}
			}
		}
	}


	// This is area IDs, not the area of the polygon.
	unsigned char* PolyAreas = (unsigned char*)FMemory::Malloc(sizeof(unsigned char) * TileParams.polyCount);
	//FMemory::Memset(PolyAreas, 0xff, sizeof(unsigned char)*MaxPolys);
	FMemory::Memset(PolyAreas, RC_WALKABLE_AREA, sizeof(unsigned char) * TileParams.polyCount);

	// 0xff seems to mean walkable. 0x00 seems to mean impassable. Recast seems to generate 0x0001 but not sure
	// what that means
	unsigned short* PolyFlags = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * TileParams.polyCount);
	//FMemory::Memset(PolyFlags, 0xff, sizeof(unsigned short) * MaxPolys);
	if (bNoFlags)
	{
		FMemory::Memset(PolyFlags, 0xff, sizeof(unsigned short) * TileParams.polyCount);
	}
	else {
		FMemory::Memset(PolyFlags, 0x0001, sizeof(unsigned short) * TileParams.polyCount);
	}

	for (auto Region : SourceMap->GetImpassableRegions())
	{
		int32* MappedPoly = PolyMap.Find(Region);
		if (MappedPoly)
		{
			PolyFlags[*MappedPoly] = 0x00;
			PolyAreas[*MappedPoly] = RC_NULL_AREA;
		}
	}

	// Off mesh connections
	TileParams.offMeshConCount = 0;
	TileParams.offMeshCons = 0;

	// Will be created by the init method based on the base poly
	TileParams.detailMeshes = 0;
	TileParams.detailVerts = 0;
	TileParams.detailVertsCount = 0;
	TileParams.detailTris = 0;
	TileParams.detailTriCount = 0;

	TileParams.verts = verts;
	TileParams.polys = polys;
	TileParams.polyAreas = PolyAreas;
	TileParams.polyFlags = PolyFlags;
}

int32 AManualDetourNavMesh::ProcessTriangle(const FBox2D& Bounds, int32 TriIndex, const FPCGDelaunayTriangulation &Triangulation, const TArray<float> &CoordHeights,
		TArray<FVector> &TmpVerts, TArray<unsigned short> &TmpPolys, int32 CurrentPoly, TMap<int32, FPCGTriPoint> &InternalVerts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts)
{
	int32 Ai = Triangulation.Triangles[3 * TriIndex];
	int32 Bi = Triangulation.Triangles[3 * TriIndex+1];
	int32 Ci = Triangulation.Triangles[3 * TriIndex+2];

	const FVector2D& A = Triangulation.Coords[Ai];
	const FVector2D& B = Triangulation.Coords[Bi];
	const FVector2D& C = Triangulation.Coords[Ci];

	// Do a quick check to see if all points are on a single side of the bounds
	if (!TriangleRelevanceCheck(A, B, C, Bounds))
	{
		UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Triangle %d with points %d %d %d failed relevance check, skipping"), TriIndex, Ai, Bi, Ci);
		return 0;
	}

	// Get the status of all the points in this triangle
	FPCGTriPoint Av = ProcessPoint(Ai, Bounds, Triangulation.Coords, CoordHeights, TmpVerts, InternalVerts);
	FPCGTriPoint Bv = ProcessPoint(Bi, Bounds, Triangulation.Coords, CoordHeights, TmpVerts, InternalVerts);
	FPCGTriPoint Cv = ProcessPoint(Ci, Bounds, Triangulation.Coords, CoordHeights, TmpVerts, InternalVerts);

	TriPointArray PointsToPlace;
	FPCGTriCorners Corners;

	// If all points are within the bounds, we can create the poly and get outta here
	if (Av.Index >= 0 && Bv.Index >= 0 && Cv.Index >= 0)
	{
		UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placing internal triangle %d at poly %d with points %d %d %d"), TriIndex, CurrentPoly, Ai, Bi, Ci);
		// TODO make poly
		PointsToPlace.Add(TriPointType(&Av, EPCGTriMember::E_A));
		PointsToPlace.Add(TriPointType(&Bv, EPCGTriMember::E_B));
		PointsToPlace.Add(TriPointType(&Cv, EPCGTriMember::E_C));

		PlacePoints(CurrentPoly, TriIndex, PointsToPlace, TmpVerts, TmpPolys, InternalVerts, HalfEdgeVerts, Corners, Triangulation, CoordHeights, Bounds);
		return 3;
	}


	// If all the corners are within this triangle we can make a quad and be done. This
	// also prevents having to handle this case when walking along edges, which would be
	// quite awkward without any obvious starting point

	FVector2D B1 = FVector2D(Bounds.Min.X, Bounds.Max.Y);
	FVector2D B2 = FVector2D(Bounds.Max.X, Bounds.Min.Y);
	Corners.SW = URegionDistribution::IsInTriangle(Bounds.Min, A, B, C);
	Corners.NE = URegionDistribution::IsInTriangle(Bounds.Max, A, B, C);
	Corners.NW = URegionDistribution::IsInTriangle(B1, A, B, C);
	Corners.SE = URegionDistribution::IsInTriangle(B2, A, B, C);
	if (Corners.IsContained())
	{
		// Triangle contains the entire tile
		// TODO make poly
		UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placing triangle %d at poly %d with all corner points"), TriIndex, CurrentPoly, Ai, Bi, Ci);
		TmpVerts.Add(FVector(Bounds.Min, GetHeightAtPointInTri(Bounds.Min, Triangulation, TriIndex, CoordHeights)));
		TmpVerts.Add(FVector(B2, GetHeightAtPointInTri(Bounds.Min, Triangulation, TriIndex, CoordHeights)));
		TmpVerts.Add(FVector(Bounds.Max, GetHeightAtPointInTri(Bounds.Min, Triangulation, TriIndex, CoordHeights)));
		TmpVerts.Add(FVector(B1, GetHeightAtPointInTri(Bounds.Min, Triangulation, TriIndex, CoordHeights)));
		TmpPolys[0] = 0;
		TmpPolys[1] = 1;
		TmpPolys[2] = 2;
		TmpPolys[3] = 3;
		TmpPolys[6 + 0] = GetEdgePortal(EPCGTriPoint::E_South);
		TmpPolys[6 + 1] = GetEdgePortal(EPCGTriPoint::E_East);
		TmpPolys[6 + 2] = GetEdgePortal(EPCGTriPoint::E_North);
		TmpPolys[6 + 3] = GetEdgePortal(EPCGTriPoint::E_West);
		return 4;
	}

	// Start testing the edges to see if there's intersections with the bounds and/or colinear segments with the bounds
	FPCGTriPoint EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC;

	ProcessEdge(TriIndex, EdgeAB, EdgeBA, 3*TriIndex, Av, Bv, Bounds, Triangulation, CoordHeights, TmpVerts, TmpPolys, CurrentPoly, InternalVerts, HalfEdgeVerts);
	ProcessEdge(TriIndex, EdgeBC, EdgeCB, 3*TriIndex+1, Bv, Cv, Bounds, Triangulation, CoordHeights, TmpVerts, TmpPolys, CurrentPoly, InternalVerts, HalfEdgeVerts);
	ProcessEdge(TriIndex, EdgeCA, EdgeAC, 3*TriIndex+2, Cv, Av, Bounds, Triangulation, CoordHeights, TmpVerts, TmpPolys, CurrentPoly, InternalVerts, HalfEdgeVerts);

	LogTriPoly(TriIndex, EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC, Av, Bv, Cv, Corners, Triangulation);
	int32 Count = PointCount(EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC, Av, Bv, Cv, Corners);

	// If there's less than 3 valid points then we might have found two edge vertices with the third pointing externally
	// which is a valid triangle but not for this tile
	if (Count < 3)
	{
		UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Triangle %d has %d points which is not enough, skipping"), TriIndex, Count);
		return Count;
	}

	// Edge intersections + Points within the bounds + Corners within the triangle
	//
	// There can be cases where points are too close to each other and need to be merged
	// which will need to be handled later, but that can only reduce the max point count

	// add the half edge int32, use the direction of the edge to determine which half edge
	// should own the point. ie. if the edge hits two points, only add the first since the pair
	// will add the other point if it exists. Use a distance sq check

	// To create the poly walk around the points and both edges:

	// - Point A
	// - HalfEdge A - > B
	// - HalfEdge B - > A
	// - Point B
	// - HalfEdge B - > C
	// - HalfEdge C - > B
	// - Point C
	// - HalfEdge C - > A
	// - HalfEdge A - > C
	//
	// Any time we cross an edge we need to track which edge it was. If we return via a different edge we need to
	// check which corners are available in order to get from the egress to the ingress point cleanly

	if (Av.Index >= 0)
		PointsToPlace.Add(TriPointType(&Av, EPCGTriMember::E_A));
	if (EdgeAB.Index >= 0)
		PointsToPlace.Add(TriPointType(&EdgeAB, EPCGTriMember::E_AB));
	if (EdgeBA.Index >= 0)
		PointsToPlace.Add(TriPointType(&EdgeBA, EPCGTriMember::E_BA));
	
	if (Bv.Index >= 0)
		PointsToPlace.Add(TriPointType(&Bv, EPCGTriMember::E_B));
	if (EdgeBC.Index >= 0)
		PointsToPlace.Add(TriPointType(&EdgeBC, EPCGTriMember::E_BC));
	if (EdgeCB.Index >= 0)
		PointsToPlace.Add(TriPointType(&EdgeCB, EPCGTriMember::E_CB));

	if (Cv.Index >= 0)
		PointsToPlace.Add(TriPointType(&Cv, EPCGTriMember::E_C));
	if (EdgeCA.Index >= 0)
		PointsToPlace.Add(TriPointType(&EdgeCA, EPCGTriMember::E_CA));
	if (EdgeAC.Index >= 0)
		PointsToPlace.Add(TriPointType(&EdgeAC, EPCGTriMember::E_AC));

	PlacePoints(CurrentPoly, TriIndex, PointsToPlace, TmpVerts, TmpPolys, InternalVerts, HalfEdgeVerts, Corners, Triangulation, CoordHeights, Bounds);
	return Count;
}

void AManualDetourNavMesh::PlacePoints(int32 PolyIndex, int32 TriIndex, TriPointArray &PointsToPlace, TArray<FVector>& TmpVerts, TArray<unsigned short>& TmpPolys, TMap<int32, FPCGTriPoint>& InternalVerts,
	TMap<int32, FPCGTriPoint>& HalfEdgeVerts, FPCGTriCorners &Corners, const FPCGDelaunayTriangulation& Triangulation, const TArray<float> &CoordHeights, const FBox2D &Bounds)
{
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placing Triangle %d into poly %d, has %d points"), TriIndex, PolyIndex, PointsToPlace.Num());
	int32 PlaceIndex = 0;
	for (int32 i = 0; i < PointsToPlace.Num(); i++)
	{
		//TriPointType& PreviousPoint = i == 0 ? PointsToPlace.Last() : PointsToPlace[i - 1];
		TriPointType& CurrentPoint = PointsToPlace[i];
		TriPointType& NextPoint = i == PointsToPlace.Num() - 1 ? PointsToPlace[0] : PointsToPlace[i + 1];

		// need the poly index so we can place in the right spot
		// Add vertex to tmppolys
		unsigned short VertIdx = (unsigned short)CurrentPoint.Key->Index;
		TmpPolys[6 * PolyIndex * 2 + PlaceIndex] = VertIdx;

		// Use the triangulation's numbering since we won't know the corrected numbering until everything has been placed
			// TODO actually we could do reciprocal updates as we go if we were able to store where each edge was stored within
			// the poly. It can't go into the halfedgemap because some (most) edges with adjacents won't be in there as they go directly
			// between two internal vertices. Perhaps another map is worth it to avoid another loop over all polys in the tile

		// if current point is a half edge, the next point is a half edge, and the edges can't connect
		// directly to each other (eg. if one of them is a corner already),
		// add corners before we move on to the next point. Corner placement will handle the adjacency 
		// for the current point since NextPoint is not actually the next point in this case.

		// TODO need to handle the triangle case where we only have eg. CA -> AC with a corner connecting them
		// The corner will be added twice unless we ensure that CA -> AC doesn't match this case
		// If travelling from HE2 to HE1, don't add the corner? Ie. don't match on AC->CA, BA->AB, CB -> BC but do match on the reverse.
		// It's the inverse : *only* create corners when travelling BA->, CB-> or AC->

		if (IsEdge(NextPoint.Value) && IsEdge(CurrentPoint.Value) && GetCommonEdge(CurrentPoint.Key->Point, NextPoint.Key->Point) == EPCGTriPoint::E_External && ReverseEdgeCheck(CurrentPoint.Value, NextPoint.Value))
		{
			// add needed corner(s) and add current edge adjacency
			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placing corners in Triangle %d, poly %d, between %d and %d"), TriIndex, PolyIndex, i, i == PointsToPlace.Num() - 1 ? 0 : i + 1);
			if (!PlaceCorners(PlaceIndex, CurrentPoint, NextPoint, PolyIndex, TriIndex, PointsToPlace, TmpVerts, TmpPolys, InternalVerts, HalfEdgeVerts, Corners, Triangulation, CoordHeights, Bounds))
			{
				UE_LOG(LogManualNavMesh, Warning, TEXT("Failed to place corners"));
				break;
			}
		}
		else {
			// Add adjacency to tmppolys for the current edge, if it exists
			EPCGTriPoint CurrentEdge = GetCommonEdge(CurrentPoint.Key->Point, NextPoint.Key->Point);
			// portal/border case
			unsigned short BorderIndex = 6 * PolyIndex * 2 + 6 + PlaceIndex;
			if (CurrentEdge != EPCGTriPoint::E_External)
			{
				UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placing edge portal in Triangle %d, poly %d, for edge %d"), TriIndex, PolyIndex, i);
				TmpPolys[BorderIndex] = GetEdgePortal(CurrentEdge);
			}
			// adjacent poly case
			else {
				// Index of the triangle in Triangulation that we border
				unsigned short AdjacentTri = GetAdjacentPoly(Triangulation, TriIndex, CurrentPoint, NextPoint);
				TmpPolys[BorderIndex] = AdjacentTri;

				UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placing adjacency in Triangle %d, poly %d, edge %d (%s-%s) connects to poly %d"), TriIndex, PolyIndex, i, *TriMemberToString(CurrentPoint.Value), *TriMemberToString(NextPoint.Value), (int32)AdjacentTri);
			}
		}

		PlaceIndex++;
	}

	LogPlacedPoly(PolyIndex, TmpPolys);

}

bool AManualDetourNavMesh::PlaceCorners(int32 &PlaceIndex, TriPointType& CurrentPoint, TriPointType& NextPoint, int32 PolyIndex, int32 TriIndex, TriPointArray& PointsToPlace, TArray<FVector>& TmpVerts, TArray<unsigned short>& TmpPolys, TMap<int32, FPCGTriPoint>& InternalVerts,
	TMap<int32, FPCGTriPoint>& HalfEdgeVerts, FPCGTriCorners& Corners, const FPCGDelaunayTriangulation& Triangulation, const TArray<float> &CoordHeights, const FBox2D &Bounds)
{
	EPCGTriPoint StartEdge = CurrentPoint.Key->Point;
	EPCGTriPoint EndEdge = NextPoint.Key->Point;

	EPCGTriPoint CurrentEdge = StartEdge;
	EPCGTriPoint NextEdge = GetNextEdge(CurrentEdge, StartEdge, EndEdge, Corners);

	// Insert adjacency for the current (half edge) point
	TmpPolys[6 * PolyIndex * 2 + 6 + PlaceIndex] = GetEdgePortal(GetCommonEdge(NextEdge, CurrentEdge));

	//EPCGTriPoint PreviousEdge = StartEdge;

	while (NextEdge != EndEdge)
	{
		PlaceIndex++;
		//PreviousEdge = CurrentEdge;

		CurrentEdge = NextEdge;

		// Get an edge connected to the currentedge towards end edge
		NextEdge = GetNextEdge(CurrentEdge, StartEdge, EndEdge, Corners);

		FPCGTriPoint CornerPoint = FPCGTriPoint(TmpVerts.Num(), CurrentEdge);
		TmpVerts.Add(CornerEdgeToVert(CurrentEdge, Bounds, TriIndex, Triangulation, CoordHeights));

		unsigned short Border = GetEdgePortal(GetCommonEdge(CurrentEdge, NextEdge));
		TmpPolys[6 * PolyIndex * 2 + 6 + PlaceIndex] = Border;

		// place vert in poly
		unsigned short VertIdx = (unsigned short)CornerPoint.Index;
		TmpPolys[6 * PolyIndex * 2 + PlaceIndex] = VertIdx;

		UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placing corner %s in triangle %d since %s is followed by %s and tri contains the corners %s"), *CornerPoint.ToString(), TriIndex, *CurrentPoint.Key->ToString(), *NextPoint.Key->ToString(), *CornerString(Corners));

		if (PlaceIndex > 4)
		{
			return false;
		}
	}

	return true;
}

FVector AManualDetourNavMesh::CornerEdgeToVert(EPCGTriPoint CornerEdge, const FBox2D& Bounds, int32 TriIndex, const FPCGDelaunayTriangulation &Triangulation, const TArray<float> &CoordHeights)
{
	switch (CornerEdge)
	{
	case EPCGTriPoint::E_NorthEast:
		return FVector(Bounds.Max, GetHeightAtPointInTri(Bounds.Max, Triangulation, TriIndex, CoordHeights));
	case EPCGTriPoint::E_SouthWest:
		return FVector(Bounds.Min, GetHeightAtPointInTri(Bounds.Min, Triangulation, TriIndex, CoordHeights));
	case EPCGTriPoint::E_NorthWest:
		return FVector(Bounds.Min.X, Bounds.Max.Y, GetHeightAtPointInTri(FVector2D(Bounds.Min.X, Bounds.Max.Y), Triangulation, TriIndex, CoordHeights));
	case EPCGTriPoint::E_SouthEast:
		return FVector(Bounds.Max.X, Bounds.Min.Y, GetHeightAtPointInTri(FVector2D(Bounds.Max.X, Bounds.Min.Y), Triangulation, TriIndex, CoordHeights));
	}
	return FVector::ZeroVector;
}

EPCGTriMember AManualDetourNavMesh::GetConnectingEdge(EPCGTriMember P1, EPCGTriMember P2)
{
	if ((P1 == EPCGTriMember::E_A && P2 == EPCGTriMember::E_B) ||
		(P1 == EPCGTriMember::E_A && P2 == EPCGTriMember::E_BA) ||
		(P1 == EPCGTriMember::E_AB && P2 == EPCGTriMember::E_B) ||
		(P1 == EPCGTriMember::E_AB && P2 == EPCGTriMember::E_BA) ||
		(P1 == EPCGTriMember::E_BA && P2 == EPCGTriMember::E_B))
	{
		return EPCGTriMember::E_AB;
	}

	if ((P1 == EPCGTriMember::E_B && P2 == EPCGTriMember::E_C) ||
		(P1 == EPCGTriMember::E_B && P2 == EPCGTriMember::E_CB) ||
		(P1 == EPCGTriMember::E_BC && P2 == EPCGTriMember::E_C) ||
		(P1 == EPCGTriMember::E_BC && P2 == EPCGTriMember::E_CB) ||
		(P1 == EPCGTriMember::E_CB && P2 == EPCGTriMember::E_C))
	{
		return EPCGTriMember::E_BC;
	}

	if ((P1 == EPCGTriMember::E_C && P2 == EPCGTriMember::E_A) ||
		(P1 == EPCGTriMember::E_C && P2 == EPCGTriMember::E_AC) ||
		(P1 == EPCGTriMember::E_CA && P2 == EPCGTriMember::E_A) ||
		(P1 == EPCGTriMember::E_CA && P2 == EPCGTriMember::E_AC) ||
		(P1 == EPCGTriMember::E_AC && P2 == EPCGTriMember::E_A))
	{
		return EPCGTriMember::E_CA;
	}

	return EPCGTriMember::E_A;
}

unsigned short AManualDetourNavMesh::ConnectingEdgeToOffset(EPCGTriMember Edge)
{
	switch (Edge)
	{
	case EPCGTriMember::E_AB:
		return 0;
	case EPCGTriMember::E_BC:
		return 1;
	case EPCGTriMember::E_CA:
		return 2;
	default:
		return RC_MESH_NULL_IDX;
	}
}

unsigned short AManualDetourNavMesh::GetAdjacentPoly(const FPCGDelaunayTriangulation& Triangulation, int32 TriIndex, TriPointType &CurrentPoint, TriPointType &NextPoint)
{
	unsigned short HalfEdgeIdx = TriIndex * 3;

	// This doesn't account for the edge pair being the provider of the point - it always looks
	// for the current triangle's halfedge.
	EPCGTriMember ConnectingEdge = GetConnectingEdge(CurrentPoint.Value, NextPoint.Value);
	unsigned short Offset = ConnectingEdgeToOffset(ConnectingEdge);

	if (Offset == RC_MESH_NULL_IDX)
	{
		return Offset;
	}

	HalfEdgeIdx += ConnectingEdgeToOffset(ConnectingEdge);
	int32 HalfEdgePair = Triangulation.HalfEdges[HalfEdgeIdx];
	if (HalfEdgePair < 0)
	{
		return RC_MESH_NULL_IDX;
	}
	else {
		return (unsigned short) Triangulation.TriangleOfEdge(HalfEdgePair);
	}

}

EPCGTriPoint AManualDetourNavMesh::GetNextEdge(EPCGTriPoint CurrentEdge, EPCGTriPoint StartEdge, EPCGTriPoint EndEdge, FPCGTriCorners& Corners)
{
	// If we're on an edge then move to a corner we're connected to
	// This only happens for the first movement
	if (CurrentEdge == EPCGTriPoint::E_East)
	{
		if (Corners.NE)
			return EPCGTriPoint::E_NorthEast;
		if (Corners.SE)
			return EPCGTriPoint::E_SouthEast;
	}

	if (CurrentEdge == EPCGTriPoint::E_West)
	{
		if (Corners.NW)
			return EPCGTriPoint::E_NorthWest;
		if (Corners.SW)
			return EPCGTriPoint::E_SouthWest;
	}

	if (CurrentEdge == EPCGTriPoint::E_North)
	{
		if (Corners.NW)
			return EPCGTriPoint::E_NorthWest;
		if (Corners.NE)
			return EPCGTriPoint::E_NorthEast;
	}

	if (CurrentEdge == EPCGTriPoint::E_South)
	{
		if (Corners.SW)
			return EPCGTriPoint::E_SouthWest;
		if (Corners.SE)
			return EPCGTriPoint::E_SouthEast;
	}

	if (CornerDistToEdge(CurrentEdge, EndEdge) == 1)
		return EndEdge;

	return GetNextCornerEdge(CurrentEdge, EndEdge, Corners);
}

EPCGTriPoint AManualDetourNavMesh::GetNextCornerEdge(EPCGTriPoint Corner, EPCGTriPoint Destination, FPCGTriCorners& Corners)
{
	EPCGTriPoint Ret = EPCGTriPoint::E_External;
	int32 RetI = 5;

	if (Corners.SW && CornerDistToEdge(EPCGTriPoint::E_SouthWest, Destination) < RetI)
	{
		Ret = EPCGTriPoint::E_SouthWest;
		RetI = CornerDistToEdge(EPCGTriPoint::E_SouthWest, Destination);
	}

	if (Corners.NW && CornerDistToEdge(EPCGTriPoint::E_NorthWest, Destination) < RetI)
	{
		Ret = EPCGTriPoint::E_NorthWest;
		RetI = CornerDistToEdge(EPCGTriPoint::E_NorthWest, Destination);
	}

	if (Corners.SE && CornerDistToEdge(EPCGTriPoint::E_SouthEast, Destination) < RetI)
	{
		Ret = EPCGTriPoint::E_SouthEast;
		RetI = CornerDistToEdge(EPCGTriPoint::E_SouthEast, Destination);
	}
	if (Corners.NE && CornerDistToEdge(EPCGTriPoint::E_NorthEast, Destination) < RetI)
	{
		Ret = EPCGTriPoint::E_NorthEast;
		RetI = CornerDistToEdge(EPCGTriPoint::E_NorthEast, Destination);
	}

	return Ret;
}

int32 AManualDetourNavMesh::CornerDistToEdge(EPCGTriPoint Corner, EPCGTriPoint Destination)
{
	if (Corner == Destination)
	{
		return 0;
	}

	if (Corner == EPCGTriPoint::E_NorthEast)
	{
		if (Destination == EPCGTriPoint::E_East || Destination == EPCGTriPoint::E_North)
			return 1;
		if (Destination == EPCGTriPoint::E_NorthWest || Destination == EPCGTriPoint::E_SouthEast)
			return 2;
		if (Destination == EPCGTriPoint::E_West || Destination == EPCGTriPoint::E_South)
			return 3;
		if (Destination == EPCGTriPoint::E_SouthWest)
			return 4;
	}

	if (Corner == EPCGTriPoint::E_NorthWest)
	{
		if (Destination == EPCGTriPoint::E_West || Destination == EPCGTriPoint::E_North)
			return 1;
		if (Destination == EPCGTriPoint::E_NorthEast || Destination == EPCGTriPoint::E_SouthWest)
			return 2;
		if (Destination == EPCGTriPoint::E_East || Destination == EPCGTriPoint::E_South)
			return 3;
		if (Destination == EPCGTriPoint::E_SouthEast)
			return 4;
	}
	if (Corner == EPCGTriPoint::E_SouthEast)
	{
		if (Destination == EPCGTriPoint::E_East || Destination == EPCGTriPoint::E_South)
			return 1;
		if (Destination == EPCGTriPoint::E_SouthWest || Destination == EPCGTriPoint::E_NorthEast)
			return 2;
		if (Destination == EPCGTriPoint::E_West || Destination == EPCGTriPoint::E_North)
			return 3;
		if (Destination == EPCGTriPoint::E_NorthWest)
			return 4;
	}
	if (Corner == EPCGTriPoint::E_SouthWest)
	{
		if (Destination == EPCGTriPoint::E_West || Destination == EPCGTriPoint::E_South)
			return 1;
		if (Destination == EPCGTriPoint::E_SouthEast || Destination == EPCGTriPoint::E_NorthWest)
			return 2;
		if (Destination == EPCGTriPoint::E_East || Destination == EPCGTriPoint::E_North)
			return 3;
		if (Destination == EPCGTriPoint::E_NorthEast)
			return 4;
	}
	return 0;
}

FPCGTriPoint AManualDetourNavMesh::ProcessPoint(int32 PointIndex, const FBox2D& Bounds, const TArray<FVector2D>& Coords, const TArray<float> &CoordHeights, TArray<FVector>& TmpVerts, TMap<int32, FPCGTriPoint>& InternalVerts)
{
	// Point is already handled for this tile
	FPCGTriPoint* V = InternalVerts.Find(PointIndex);
	if (V)
	{
		return *V;
	}

	/*
		For any point, it's inside the bounds, on the edge, is a corner or is external. In the first 3 cases add the point to TmpVerts and add a mapping from
		PointIndex to the new Index in TmpVerts.
	*/
	//const FVector2D& TestPoint = Coords[PointIndex];
	const FVector& TestPoint = FVector(Coords[PointIndex], CoordHeights[PointIndex]);
	if ((TestPoint.X >= Bounds.Min.X) && (TestPoint.X <= Bounds.Max.X) && (TestPoint.Y >= Bounds.Min.Y) && (TestPoint.Y <= Bounds.Max.Y))
	{
		int32 PV = TmpVerts.Num();
		TmpVerts.Add(TestPoint);
		FPCGTriPoint TP = FPCGTriPoint(PV, EPCGTriPoint::E_External);

		// Internal point
		if ((TestPoint.X > Bounds.Min.X) && (TestPoint.X < Bounds.Max.X) && (TestPoint.Y > Bounds.Min.Y) && (TestPoint.Y < Bounds.Max.Y))
		{
			TP = FPCGTriPoint(PV, EPCGTriPoint::E_Internal);
		}

		// Edge and corner points
		else if (TestPoint.X == Bounds.Min.X)
		{
			if (TestPoint.Y == Bounds.Min.Y)
			{
				TP = FPCGTriPoint(PV, EPCGTriPoint::E_SouthWest);
			}
			else if (TestPoint.Y == Bounds.Max.Y)
			{
				TP = FPCGTriPoint(PV, EPCGTriPoint::E_NorthWest);
			}
			else {
				TP = FPCGTriPoint(PV, EPCGTriPoint::E_West);
			}
		}

		else if (TestPoint.X == Bounds.Max.X)
		{
			if (TestPoint.Y == Bounds.Min.Y)
			{
				TP = FPCGTriPoint(PV, EPCGTriPoint::E_SouthEast);
			}
			else if (TestPoint.Y == Bounds.Max.Y)
			{
				TP = FPCGTriPoint(PV, EPCGTriPoint::E_NorthEast);
			}
			else {
				TP = FPCGTriPoint(PV, EPCGTriPoint::E_East);
			}
		}

		else if (TestPoint.Y == Bounds.Min.Y)
		{
			TP = FPCGTriPoint(PV, EPCGTriPoint::E_South);
		}

		else if (TestPoint.Y == Bounds.Max.Y)
		{
			TP = FPCGTriPoint(PV, EPCGTriPoint::E_North);
		}

		if (TP.Point == EPCGTriPoint::E_External)
		{
			UE_LOG(LogManualNavMesh, Warning, TEXT("point %d (%s) lies within the border but wasn't assigned an internal code"), PointIndex, *TestPoint.ToString());
		}

		InternalVerts.Add(PointIndex, TP);
		return TP;
	}

	// Point is not within the bounds of this tile
	return FPCGTriPoint(-1, EPCGTriPoint::E_External);
}

void AManualDetourNavMesh::ProcessEdge(int32 TriIndex, FPCGTriPoint& Edge1, FPCGTriPoint& Edge2, int32 EdgeIndex, const FPCGTriPoint& A, const FPCGTriPoint& B, const FBox2D& Bounds,
	const FPCGDelaunayTriangulation& Triangulation, const TArray<float> &CoordHeights, TArray<FVector>& TmpVerts, TArray<unsigned short>& TmpPolys, int32& CurrentPoly,
	TMap<int32, FPCGTriPoint>& InternalVerts, TMap<int32, FPCGTriPoint>& HalfEdgeVerts)
{

	/*
		There's quite a few cases for edges.
		 - Is colinear with a boundary segment
		   - With one point outside bounds ( outside -> inside edge has the corner )
		   - With both points outside bounds ( both edges have corners )
		   - With both points inside bounds ( both edges are ignored since the points will already be included )
		 - Intersects two boundary segments ( both edges will have an edge point )
		 - Intersects one boundary segment ( outside -> inside edge has the edge point )

		 In some cases the paired edge will not exist in HalfEdges but does need to map to a point. This will happen
		 when the edge forms the boundary of the triangulation, and can commonly happen with the case where both edges need to
		 be corners.
		 This shouldn't be a problem because the edge of the triangulation will have no adjacent polys, so we don't need to be
		 able to track these verts from another poly.
	*/

	/*
		A Key of -1 indicates nothing is here since it's meant to be the value of this point in TmpVerts.
	*/
	Edge1.Index = -1;
	Edge2.Index = -1;
	Edge1.Point = EPCGTriPoint::E_External;
	Edge2.Point = EPCGTriPoint::E_External;

	// The very first thing to test is colinear case 3, since we want to ignore all checks on those types of segments and just use the
	// points as-is without adjustment, and they'll cause problems with both colinear tests and intersection tests.
	if (IsInternalSegment(A, B))
	{
		return;
	}

	// Next we can check if this edge has already been walked by a neighbour triangle - we can use their results instead of recalculating
	FPCGTriPoint* E1 = HalfEdgeVerts.Find(EdgeIndex);
	if (E1)
	{
		Edge1.Index = E1->Index;
		Edge1.Point = E1->Point;
	}

	int32 Edge2Index = Triangulation.HalfEdges[EdgeIndex] >= 0 ? Triangulation.HalfEdges[EdgeIndex] : -1;
	if (Edge2Index >= 0)
	{
		FPCGTriPoint* E2 = HalfEdgeVerts.Find(Edge2Index);
		if (E2)
		{
			Edge2.Index = E2->Index;
			Edge2.Point = E2->Point;
		}
	}

	// Edge was already processed by our neighbour
	// TODO add an unprocessed option to the ENUM so we can track edges that didn't match
	// as well and prevent double-testing everything that isn't relevant
	if (Edge1.Index >= 0 || Edge2.Index >= 0)
	{
		return;
	}

	// We probably want to test colinear cases next because the intersection tests will give positive results but they
	// will be wrong if the segment is colinear
	int32 VertAi = Triangulation.Triangles[EdgeIndex];
	int32 VertBi = Triangulation.Triangles[Triangulation.NextHalfedge(EdgeIndex)];

	FVector2D VertA = Triangulation.Coords[VertAi];
	FVector2D VertB = Triangulation.Coords[VertBi];

	ColinearEdgeCheck(TriIndex, Edge1, EdgeIndex, A, B, VertA, VertB, Bounds, Triangulation, CoordHeights, TmpVerts, HalfEdgeVerts);
	ColinearEdgeCheck(TriIndex, Edge2, Edge2Index, B, A, VertB, VertA, Bounds, Triangulation, CoordHeights, TmpVerts, HalfEdgeVerts);

	// Edge was colinear, we are done 
	if (Edge1.Index >= 0 || Edge2.Index >= 0)
	{
		return;
	}

	// Intersection test. Only return positive result when A is external and B is internal, or return the
	// closer result to A if both are external.
	IntersectingEdgeCheck(TriIndex, Edge1, EdgeIndex, A, B, VertA, VertB, Bounds, Triangulation, CoordHeights, TmpVerts, HalfEdgeVerts);
	IntersectingEdgeCheck(TriIndex, Edge2, Edge2Index, B, A, VertB, VertA, Bounds, Triangulation, CoordHeights, TmpVerts, HalfEdgeVerts);

}

void AManualDetourNavMesh::ColinearEdgeCheck(int32 TriIndex, FPCGTriPoint &Edge, int32 EdgeIndex, const FPCGTriPoint &PA, const FPCGTriPoint &PB, FVector2D &VA, FVector2D &VB, const FBox2D& Bounds,
	const FPCGDelaunayTriangulation &Triangulation, const TArray<float> &CoordHeights,
		TArray<FVector> &TmpVerts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts)
{

	if (VA.X == VB.X)
	{
		// AB Colinear with East Boundary
		if (VA.X == Bounds.Max.X)
		{
			if (VA.Y < Bounds.Min.Y && VB.Y > Bounds.Min.Y)
			{
				// A is South of the East boundary and B is either on or North of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Max.X, Bounds.Min.Y), Edge, TriIndex, EdgeIndex, EPCGTriPoint::E_SouthEast, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
			else if (VA.Y > Bounds.Max.Y && VB.Y < Bounds.Max.Y)
			{
				// A is North of the East boundary and B is either on or South of it
				AddHalfEdgeTempVert(Bounds.Max, Edge,TriIndex, EdgeIndex, EPCGTriPoint::E_NorthEast, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
		}
		// AB Colinear with West Boundary
		else if (VA.X == Bounds.Min.X)
		{
			if (VA.Y < Bounds.Min.Y && VB.Y > Bounds.Min.Y)
			{
				// A is South of the West boundary and B is either on or North of it
				AddHalfEdgeTempVert(Bounds.Min, Edge,TriIndex, EdgeIndex, EPCGTriPoint::E_SouthWest, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
			else if (VA.Y > Bounds.Max.Y && VB.Y < Bounds.Max.Y)
			{
				// A is North of the West boundary and B is either on or South of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Min.X, Bounds.Max.Y), Edge,TriIndex, EdgeIndex, EPCGTriPoint::E_NorthWest, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
		}
	}

	if (VA.Y == VB.Y)
	{
		// AB Colinear with North Boundary
		if (VA.Y == Bounds.Max.Y)
		{
			if (VA.X < Bounds.Min.X && VB.X > Bounds.Min.X)
			{
				// A is West of the North boundary and B is either on or East of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Min.X, Bounds.Max.Y), Edge,TriIndex, EdgeIndex, EPCGTriPoint::E_NorthWest, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
			else if (VA.X > Bounds.Max.X && VB.X < Bounds.Max.X)
			{
				// A is East of the North boundary and B is either on or West of it
				AddHalfEdgeTempVert(Bounds.Max, Edge,TriIndex, EdgeIndex, EPCGTriPoint::E_NorthEast, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
		}
		// AB Colinear with South Boundary
		else if (VA.Y == Bounds.Min.Y)
		{
			if (VA.X < Bounds.Min.X && VB.X > Bounds.Min.X)
			{
				// A is West of the South boundary and B is either on or East of it
				AddHalfEdgeTempVert(Bounds.Min, Edge,TriIndex, EdgeIndex, EPCGTriPoint::E_SouthWest, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
			else if (VA.X > Bounds.Max.X && VB.X < Bounds.Max.X)
			{
				// A is East of the South boundary and B is either on or West of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Max.X, Bounds.Min.Y), Edge,TriIndex, EdgeIndex, EPCGTriPoint::E_SouthEast, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
		}
	}

}
void AManualDetourNavMesh::IntersectingEdgeCheck(int32 TriIndex, FPCGTriPoint& Edge, int32 EdgeIndex, const FPCGTriPoint& PA, const FPCGTriPoint& PB, FVector2D& VA, FVector2D& VB, const FBox2D& Bounds,
	const FPCGDelaunayTriangulation& Triangulation, const TArray<float> &CoordHeights,
	TArray<FVector>& TmpVerts, TMap<int32, FPCGTriPoint>& HalfEdgeVerts)
{
	// TODO the intersection test should also give us the height for the intersect point

	// Intersection test. Only return positive result when A is external and B is internal, or return the
	// closer result to A if both are external.
	if (PA.Point != EPCGTriPoint::E_External)
	{
		return;
	}

	FVector B1 = FVector(Bounds.Min.X, Bounds.Max.Y, 0.f);
	FVector B2 = FVector(Bounds.Max.X, Bounds.Min.Y, 0.f);

	// Note that we don't want cases where B is on the edge, only actual internal points give intersections 
	// WRONG! we could have B on the edge or in a corner and the external all the way on the other side
	// of the tile -_-
	// This complicates things because the intersect point we want in these cases won't be unique, we'll have
	// to filter out the intersections the corner has with its own edges, for example (or not test at all?)

	// TODO this might need to be corrected to the corner in the unlikely event the segment intersects with the corner
	if (PA.Point == EPCGTriPoint::E_External && PB.Point != EPCGTriPoint::E_External)
	{
		/*
		DrawDebugLine(GetWorld(), FVector(VA, 0.f), FVector(VB, 0.f), FColor::Purple, true, 999.f, 0, 2.f);
		DrawDebugLine(GetWorld(), FVector(Bounds.Min, 0.f), B1, FColor::Red, true, 999.f, 0, 2.f);
		DrawDebugLine(GetWorld(), FVector(Bounds.Max, 0.f), B1, FColor::Orange, true, 999.f, 0, 2.f);
		DrawDebugLine(GetWorld(), FVector(Bounds.Max, 0.f), B2, FColor::Yellow, true, 999.f, 0, 2.f);
		DrawDebugLine(GetWorld(), FVector(Bounds.Min, 0.f), B2, FColor::Green, true, 999.f, 0, 2.f);
		*/

		FVector IntersectPoint;

		if (PB.Point != EPCGTriPoint::E_West && PB.Point != EPCGTriPoint::E_NorthWest && PB.Point != EPCGTriPoint::E_SouthWest)
		{
			if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B1, IntersectPoint))
			{
				AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, TriIndex, EdgeIndex, EPCGTriPoint::E_West, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
				return;
			}
		}
		if (PB.Point != EPCGTriPoint::E_North && PB.Point != EPCGTriPoint::E_NorthWest && PB.Point != EPCGTriPoint::E_NorthEast)
		{
			if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B1, IntersectPoint))
			{
				AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, TriIndex, EdgeIndex, EPCGTriPoint::E_North, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
				return;
			}
		}
		if (PB.Point != EPCGTriPoint::E_East && PB.Point != EPCGTriPoint::E_SouthEast && PB.Point != EPCGTriPoint::E_NorthEast)
		{
			if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B2, IntersectPoint))
			{
				AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, TriIndex, EdgeIndex, EPCGTriPoint::E_East, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
				return;
			}
		}
		if (PB.Point != EPCGTriPoint::E_South && PB.Point != EPCGTriPoint::E_SouthEast && PB.Point != EPCGTriPoint::E_SouthWest)
		{
			if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B2, IntersectPoint))
			{
				AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, TriIndex, EdgeIndex, EPCGTriPoint::E_South, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
				return;
			}
		}

	}

	// Will either be 2 intersections and pick the closest, or none at all.
	if (PA.Point == EPCGTriPoint::E_External && PB.Point == EPCGTriPoint::E_External)
	{
		FVector IntersectPoint1;
		FVector IntersectPoint2;
		EPCGTriPoint IP1Type = EPCGTriPoint::E_External;
		EPCGTriPoint IP2Type = EPCGTriPoint::E_External;

		FVector CurrentPoint;

		bool bFoundP1 = false;
		bool bFoundP2 = false;

		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B1, CurrentPoint))
		{
			bFoundP1 = true;
			IntersectPoint1 = CurrentPoint;
			IP1Type = EPCGTriPoint::E_West;
		}

		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B1, CurrentPoint))
		{

			if (bFoundP1)
			{
				bFoundP2 = true;
				IntersectPoint2 = CurrentPoint;
				IP2Type = EPCGTriPoint::E_North;
			}
			else {
				bFoundP1 = true;
				IntersectPoint1 = CurrentPoint;
				IP1Type = EPCGTriPoint::E_North;
			}

		}

		if (!bFoundP2 && FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B2, CurrentPoint))
		{
			if (bFoundP1)
			{
				bFoundP2 = true;
				IntersectPoint2 = CurrentPoint;
				IP2Type = EPCGTriPoint::E_East;
			}
			else {
				bFoundP1 = true;
				IntersectPoint1 = CurrentPoint;
				IP1Type = EPCGTriPoint::E_East;
			}
		}

		if (!bFoundP2 && FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B2, CurrentPoint))
		{
			if (bFoundP1)
			{
				bFoundP2 = true;
				IntersectPoint2 = CurrentPoint;
				IP2Type = EPCGTriPoint::E_South;
			}
			else {
				bFoundP1 = true;
				IntersectPoint1 = CurrentPoint;
				IP1Type = EPCGTriPoint::E_South;
			}
		}

		if (bFoundP2)
		{
			FVector2D P12D = FVector2D(IntersectPoint1);
			FVector2D P22D = FVector2D(IntersectPoint2);

			float DistAtoP1Sq = FVector2D::DistSquared(VA, P12D);
			float DistAtoP2Sq = FVector2D::DistSquared(VA, P22D);

			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Double connected edge index %d types %s and %s, selected %s"), EdgeIndex, *TriPointToString(IP1Type), *TriPointToString(IP2Type), DistAtoP1Sq < DistAtoP2Sq ? *TriPointToString(IP1Type) : *TriPointToString(IP2Type));

			if (DistAtoP1Sq > DistAtoP2Sq)
			{
				AddHalfEdgeTempVert(P22D, Edge, TriIndex, EdgeIndex, IP2Type, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
			else {
				AddHalfEdgeTempVert(P12D, Edge, TriIndex, EdgeIndex, IP1Type, TmpVerts, HalfEdgeVerts, Triangulation, CoordHeights, Bounds, VA, VB);
			}
		}
	}
}

void AManualDetourNavMesh::AddHalfEdgeTempVert(FVector2D Location, FPCGTriPoint& Point, int32 TriIndex, int32 EdgeIndex, EPCGTriPoint Type, TArray<FVector> &Verts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts,
		const FPCGDelaunayTriangulation &Triangulation, const TArray<float> &CoordHeights, const FBox2D &Bounds, FVector2D &VA, FVector2D &VB)
{
	//if (P.X == Bounds.Max.X || P.X == Bounds.Min.X || P.Y == Bounds.Max.Y || P.Y == Bounds.Min.Y)
	//{
	//	return GetHeightAtEdgePoint(P, Triangulation, Triangle, CoordHeights, Bounds);
	//}
	// TODO If this is a half edge vert born of an intersection, we should use the segment to find the height instead of doing a barycentric calculation
	if (Type == EPCGTriPoint::E_East || Type == EPCGTriPoint::E_North || Type == EPCGTriPoint::E_West || Type == EPCGTriPoint::E_South)
	{
		
	}

	// If this is a half edge vert that's a corner correction, use a barycentric calc since it's likely inside a triangle. If it's not, hopefully nothing explodes
	float Height = GetHeightAtPointInTri(Location, Triangulation, TriIndex, CoordHeights);
	Point.Index = Verts.Num();
	Point.Point = Type;
	Verts.Add(FVector(Location, Height));
	if (EdgeIndex >= 0)
	{
		HalfEdgeVerts.Add(EdgeIndex, Point);
	}
}

bool AManualDetourNavMesh::IsInternalSegment(const FPCGTriPoint& A, const FPCGTriPoint& B)
{
	/*
		Check each boundary
	*/
	if ((A.Point == EPCGTriPoint::E_East || A.Point == EPCGTriPoint::E_NorthEast || A.Point == EPCGTriPoint::E_SouthEast) &&
		(B.Point == EPCGTriPoint::E_East || B.Point == EPCGTriPoint::E_NorthEast || B.Point == EPCGTriPoint::E_SouthEast))
	{
		return true;
	}

	if ((A.Point == EPCGTriPoint::E_West || A.Point == EPCGTriPoint::E_NorthWest || A.Point == EPCGTriPoint::E_SouthWest) &&
		(B.Point == EPCGTriPoint::E_West || B.Point == EPCGTriPoint::E_NorthWest || B.Point == EPCGTriPoint::E_SouthWest))
	{
		return true;
	}

	if ((A.Point == EPCGTriPoint::E_South || A.Point == EPCGTriPoint::E_SouthEast || A.Point == EPCGTriPoint::E_SouthWest) &&
		(B.Point == EPCGTriPoint::E_South || B.Point == EPCGTriPoint::E_SouthEast || B.Point == EPCGTriPoint::E_SouthWest))
	{
		return true;
	}

	if ((A.Point == EPCGTriPoint::E_North || A.Point == EPCGTriPoint::E_NorthEast || A.Point == EPCGTriPoint::E_NorthWest) &&
		(B.Point == EPCGTriPoint::E_North || B.Point == EPCGTriPoint::E_NorthEast || B.Point == EPCGTriPoint::E_NorthWest))
	{
		return true;
	}
	return false;
}

bool AManualDetourNavMesh::TriangleRelevanceCheck(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FBox2D& Bounds)
{
	if (
		(A.X > Bounds.Max.X &&
			B.X > Bounds.Max.X &&
			C.X > Bounds.Max.X) ||

		(A.X < Bounds.Min.X &&
			B.X < Bounds.Min.X &&
			C.X < Bounds.Min.X) ||

		(A.Y > Bounds.Max.Y &&
			B.Y > Bounds.Max.Y &&
			C.Y > Bounds.Max.Y) ||

		(A.Y < Bounds.Min.Y &&
			B.Y < Bounds.Min.Y &&
			C.Y < Bounds.Min.Y)
		)
	{
		return false;
	}
	return true;
}

FString FPCGTriPoint::ToString() const
{
	FString Ret = FString::FromInt(Index);
	Ret.AppendChar(' ');

	switch (Point)
	{
	case EPCGTriPoint::E_External:
		Ret.Append("External");
		break;
	case EPCGTriPoint::E_Internal:
		Ret.Append("Internal");
		break;
	case EPCGTriPoint::E_NorthWest:
		Ret.Append("NorthWest");
		break;
	case EPCGTriPoint::E_North:
		Ret.Append("North");
		break;
	case EPCGTriPoint::E_NorthEast:
		Ret.Append("NorthEast");
		break;
	case EPCGTriPoint::E_East:
		Ret.Append("East");
		break;
	case EPCGTriPoint::E_SouthEast:
		Ret.Append("SouthEast");
		break;
	case EPCGTriPoint::E_South:
		Ret.Append("South");
		break;
	case EPCGTriPoint::E_SouthWest:
		Ret.Append("SouthWest");
		break;
	case EPCGTriPoint::E_West:
		Ret.Append("West");
		break;
	}

	return Ret;
}

FString AManualDetourNavMesh::CornerString(FPCGTriCorners &Corners)
{
	FString Ret = FString();
	if (Corners.NE)
		Ret.Append(TEXT("NE "));
	if (Corners.NW)
		Ret.Append(TEXT("NW "));
	if (Corners.SE)
		Ret.Append(TEXT("SE "));
	if (Corners.SW)
		Ret.Append(TEXT("SW "));
	return Ret;
}

int32 AManualDetourNavMesh::PointCount(FPCGTriPoint& EdgeAB, FPCGTriPoint& EdgeBA, FPCGTriPoint& EdgeBC, FPCGTriPoint& EdgeCB, FPCGTriPoint& EdgeCA, FPCGTriPoint& EdgeAC, FPCGTriPoint& A, FPCGTriPoint& B, FPCGTriPoint& C, FPCGTriCorners& Corners)
{
	int32 Count = 0;
	Count += PointAccum(EdgeAB, Corners);
	Count += PointAccum(EdgeBA, Corners);
	Count += PointAccum(EdgeBC, Corners);
	Count += PointAccum(EdgeCB, Corners);
	Count += PointAccum(EdgeCA, Corners);
	Count += PointAccum(EdgeAC, Corners);
	Count += PointAccum(A, Corners);
	Count += PointAccum(B,Corners);
	Count += PointAccum(C, Corners);

	if (Corners.NE)
	    Count++;
	if (Corners.NW)
	    Count++;
	if (Corners.SE)
	    Count++;
	if (Corners.SW)
	    Count++;

	return Count;
}

int32 AManualDetourNavMesh::PointAccum(FPCGTriPoint& Point, FPCGTriCorners& Corners)
{
	if (Point.Index >= 0)
	{
		if (Point.Point == EPCGTriPoint::E_NorthEast)
			Corners.NE = false;
		if (Point.Point == EPCGTriPoint::E_SouthEast)
			Corners.SE = false;
		if (Point.Point == EPCGTriPoint::E_SouthWest)
			Corners.SW = false;
		if (Point.Point == EPCGTriPoint::E_NorthWest)
			Corners.NW = false;
		return 1;
	}
	return 0;
}

FString AManualDetourNavMesh::TriPointToString(EPCGTriPoint TriPoint) const
{

	switch (TriPoint)
	{
	case EPCGTriPoint::E_External:
		return FString("External");
	case EPCGTriPoint::E_Internal:
		return FString("Internal");
	case EPCGTriPoint::E_NorthWest:
		return FString("NorthWest");
	case EPCGTriPoint::E_North:
		return FString("North");
	case EPCGTriPoint::E_NorthEast:
		return FString("NorthEast");
	case EPCGTriPoint::E_East:
		return FString("East");
	case EPCGTriPoint::E_SouthEast:
		return FString("SouthEast");
	case EPCGTriPoint::E_South:
		return FString("South");
	case EPCGTriPoint::E_SouthWest:
		return FString("SouthWest");
	case EPCGTriPoint::E_West:
		return FString("West");
	}
	return FString();
}

FString AManualDetourNavMesh::TriMemberToString(EPCGTriMember TriMember) const
{
	switch (TriMember)
	{
	case EPCGTriMember::E_A:
		return FString("A");
	case EPCGTriMember::E_B:
		return FString("B");
	case EPCGTriMember::E_C:
		return FString("C");
	case EPCGTriMember::E_AB:
		return FString("AB");
	case EPCGTriMember::E_BA:
		return FString("BA");
	case EPCGTriMember::E_BC:
		return FString("BC");
	case EPCGTriMember::E_CB:
		return FString("CB");
	case EPCGTriMember::E_CA:
		return FString("CA");
	case EPCGTriMember::E_AC:
		return FString("AC");
	}
	return FString();
}

void AManualDetourNavMesh::LogPlacedPoly(int32 Index, TArray<unsigned short>& Polys)
{
	int32 StartIndex = Index * 6 * 2;
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Placed Poly: %hu %hu %hu %hu %hu %hu | %hu %hu %hu %hu %hu %hu"),
		Polys[StartIndex], Polys[StartIndex + 1], Polys[StartIndex + 2], Polys[StartIndex + 3], Polys[StartIndex + 4], Polys[StartIndex + 5],
		Polys[StartIndex + 6], Polys[StartIndex + 7], Polys[StartIndex + 8], Polys[StartIndex + 9], Polys[StartIndex + 10], Polys[StartIndex + 11]);
		
}

void AManualDetourNavMesh::LogTriPoly(int32 Index, FPCGTriPoint &EdgeAB, FPCGTriPoint &EdgeBA,FPCGTriPoint &EdgeBC,FPCGTriPoint &EdgeCB,FPCGTriPoint &EdgeCA,FPCGTriPoint &EdgeAC,
	FPCGTriPoint &A, FPCGTriPoint &B, FPCGTriPoint &C, FPCGTriCorners& Corners, const FPCGDelaunayTriangulation &Triangulation)
{
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Processing triangle %d, PointCount: %d"), Index, PointCount(EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC, A, B, C, Corners));
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Point A (%d) : %s :: Point B (%d) : %s :: Point C (%d) %s"), Triangulation.Triangles[3*Index], *A.ToString(),Triangulation.Triangles[3*Index+1],*B.ToString(), Triangulation.Triangles[3*Index+2], *C.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge AB (%d) : %s :: Edge BA (%d) : %s"), GetHalfEdge(Index, EPCGTriMember::E_AB, Triangulation), *EdgeAB.ToString(), GetHalfEdge(Index, EPCGTriMember::E_BA, Triangulation), *EdgeBA.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge BC (%d) : %s :: Edge CB (%d) : %s"), GetHalfEdge(Index, EPCGTriMember::E_BC, Triangulation), *EdgeBC.ToString(), GetHalfEdge(Index, EPCGTriMember::E_CB, Triangulation), *EdgeCB.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge CA (%d) : %s :: Edge AC (%d) : %s"), GetHalfEdge(Index, EPCGTriMember::E_CA, Triangulation), *EdgeCA.ToString(), GetHalfEdge(Index, EPCGTriMember::E_AC, Triangulation), *EdgeAC.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Corners: %s"), *CornerString(Corners));
}

bool AManualDetourNavMesh::IsBorder(unsigned short Adjacency)
{
	if (Adjacency == 0x8000 ||
		Adjacency == 0x8001 ||
		Adjacency == 0x8002 ||
		Adjacency == 0x8003 ||
		Adjacency == RC_MESH_NULL_IDX)
	{
		return true;
	}
	return false;
}

int32 AManualDetourNavMesh::GetHalfEdge(int32 Index, EPCGTriMember Member, const FPCGDelaunayTriangulation& Triangulation)
{
	switch (Member)
	{
	case EPCGTriMember::E_AB:
		return Index * 3;
	case EPCGTriMember::E_BC:
		return Index * 3 + 1;
	case EPCGTriMember::E_CA:
		return Index * 3 + 2;

	case EPCGTriMember::E_BA:
		if (Triangulation.HalfEdges[Index * 3] < 0)
		{
			return -1;
		}
		else {
			return Triangulation.HalfEdges[Index * 3];
		}

	case EPCGTriMember::E_CB:
		if (Triangulation.HalfEdges[Index * 3 + 1] < 0)
		{
			return -1;
		}
		else {
			return Triangulation.HalfEdges[Index * 3 + 1];
		}

	case EPCGTriMember::E_AC:
		if (Triangulation.HalfEdges[Index * 3 + 2] < 0)
		{
			return -1;
		}
		else {
			return Triangulation.HalfEdges[Index * 3 + 2];
		}
	}
	return -1;
}

bool AManualDetourNavMesh::ReverseEdgeCheck(EPCGTriMember E1, EPCGTriMember E2)
{
	/*
		Check whether the edge from E1 to E2 should be allowed to place corners. This helps to prevent the case where an edge
		intersects two different boundaries, and the point placement thinks it's just placing the same edge in both directiions,
		with both attempting to pull in the corner. Instead we only want the reverse edge to pull in the corner as the forward edge
		will already be in the correct direction
	*/
	if ((E1 == EPCGTriMember::E_AB && E2 == EPCGTriMember::E_BA) ||
	    (E1 == EPCGTriMember::E_BC && E2 == EPCGTriMember::E_CB) ||
	    (E1 == EPCGTriMember::E_CA && E2 == EPCGTriMember::E_AC))
	{
		return false;
	}
	return true;
}

void AManualDetourNavMesh::ReversePolyVertOrder(int32 PolyIndex, int32 PolyVertCount, TArray<unsigned short>& TmpPolys)
{
	int32 StartVertIndex = PolyIndex * 12;
	int32 LastVertIndex = PolyIndex * 12 + PolyVertCount - 1;
	int32 PolyVertSwaps = (PolyVertCount) / 2;

	int32 StartEdgeIndex = StartVertIndex + 6;
	int32 LastEdgeIndex = LastVertIndex + 6;
	int32 PolyEdgeSwaps = (PolyVertCount - 1) / 2;

	for (int32 i = 0; i < PolyVertSwaps; i++)
	{
		Swap(TmpPolys[StartVertIndex + i], TmpPolys[LastVertIndex - i]);

		if (i < PolyEdgeSwaps)
		{
			Swap(TmpPolys[StartEdgeIndex + i], TmpPolys[LastEdgeIndex - 1 - i]);
		}
	}
}

float AManualDetourNavMesh::GetHeightAtPointInTri(const FVector2D& P, const FPCGDelaunayTriangulation& Triangulation, int32 Triangle, const TArray<float>& CoordHeights)
{

	FVector A = FVector(Triangulation.Coords[Triangulation.Triangles[3 * Triangle]], 0.f);
	FVector B = FVector(Triangulation.Coords[Triangulation.Triangles[3 * Triangle + 1]], 0.f);
	FVector C = FVector(Triangulation.Coords[Triangulation.Triangles[3 * Triangle + 2]], 0.f);


	float AH = CoordHeights[Triangulation.Triangles[3 * Triangle]];
	float BH = CoordHeights[Triangulation.Triangles[3 * Triangle + 1]];
	float CH = CoordHeights[Triangulation.Triangles[3 * Triangle + 2]];

	if (P.X == A.X && P.Y == A.Y)
		return AH;
	if (P.X == B.X && P.Y == B.Y)
		return BH;
	if (P.X == C.X && P.Y == C.Y)
		return CH;

	FVector PV = FVector(P, 0.f);
	// Check if the point is on any of the segments of the triangle
	float Ratio = GetRatioFromTriSegment(PV, B, A, EdgeHeightTolerance);
	if (Ratio >= 0.f)
	{
		DrawDebugSphere(GetWorld(), FVector(P, Ratio * AH + (1.f - Ratio) * BH), 10.f, 4, FColor::Orange, true, 999.f, 0, 1.f);
		return Ratio * AH + (1.f - Ratio) * BH;
	}
	Ratio = GetRatioFromTriSegment(PV, C, B, EdgeHeightTolerance);
	if (Ratio >= 0.f)
	{
		DrawDebugSphere(GetWorld(), FVector(P, Ratio * BH + (1.f - Ratio) * CH), 10.f, 4, FColor::Orange, true, 999.f, 0, 1.f);
		return Ratio * BH + (1.f - Ratio) * CH;
	}
	Ratio = GetRatioFromTriSegment(PV, A, C, EdgeHeightTolerance);
	if (Ratio >= 0.f)
	{
		DrawDebugSphere(GetWorld(), FVector(P,Ratio * CH + (1.f - Ratio) * AH), 10.f, 4, FColor::Orange, true, 999.f, 0, 1.f);
		return Ratio * CH + (1.f - Ratio) * AH;
	}

	FVector BaryCentric = GetRoundedBaryCentric(P, A, B, C, BaryCentricTolerance);
	float Ret = BaryCentric.X * AH +
		BaryCentric.Y * BH +
		BaryCentric.Z * CH;

	if (!ValidBaryCentric(BaryCentric.X, BaryCentric.Y, BaryCentric.Z))
	{
		DrawDebugLine(GetWorld(), A, B, FColor::Red, true, 999.f, 0, 1.f);
		DrawDebugLine(GetWorld(), B, C, FColor::Red, true, 999.f, 0, 1.f);
		DrawDebugLine(GetWorld(), C, A, FColor::Red, true, 999.f, 0, 1.f);
		DrawDebugSphere(GetWorld(), FVector(P, Ret), 10.f, 4, FColor::Purple, true, 999.f, 0, 1.f);
	}
	else {
		DrawDebugSphere(GetWorld(), FVector(P, Ret), 10.f, 4, FColor::Blue, true, 999.f, 0, 1.f);
	}
	return Ret;

}

float AManualDetourNavMesh::GetHeightAtEdgePoint(const FVector2D& P, const FPCGDelaunayTriangulation& Triangulation, int32 Triangle, const TArray<float>& CoordHeights, const FBox2D& Bounds)
{
	// TODO
	FVector A = FVector(Triangulation.Coords[Triangulation.Triangles[Triangle]], 0.f);
	FVector B = FVector(Triangulation.Coords[Triangulation.Triangles[Triangle + 1]], 0.f);
	FVector C = FVector(Triangulation.Coords[Triangulation.Triangles[Triangle + 2]], 0.f);
	FVector BaryCentric = FMath::ComputeBaryCentric2D(FVector(P, 0.f), A, B, C);
	return BaryCentric.X * CoordHeights[Triangulation.Triangles[Triangle]] +
		BaryCentric.Y * CoordHeights[Triangulation.Triangles[Triangle + 1]] +
		BaryCentric.Z * CoordHeights[Triangulation.Triangles[Triangle + 2]];
}

FVector AManualDetourNavMesh::GetRoundedBaryCentric(FVector2D Location, FVector A, FVector B, FVector C, float Tolerance)
{
	if (A == B && B == C)
	{

		UE_LOG(LogManualNavMesh, Warning, TEXT("Tried to find barycentric coods in triangle with area 0"));
		return FVector::ZeroVector;
	}

	FVector Ret;
	FVector V0 = B - A;
	FVector V1 = C - A;
	FVector V2 = FVector(Location, 0.f) - A;

	/*
	// Dot product method
	float D00 = FVector2D::DotProduct(V0, V0);
	float D01 = FVector2D::DotProduct(V0, V1);
	float D11 = FVector2D::DotProduct(V1, V1);
	float D20 = FVector2D::DotProduct(V2, V0);
	float D21 = FVector2D::DotProduct(V2, V1);
	float Denom = D00 * D11 - D01 * D01;
	v = (D11 * D20 - D01 * D21) / Denom;
	w = (D00 * D21 - D01 * D20) / Denom;
	u = 1.0f - v - w;
	*/

	float den = 1 / (V0.X * V1.Y - V1.X * V0.Y);
	Ret.Y = (V2.X * V1.Y - V1.X * V2.Y) * den;
	Ret.Z = (V0.X * V2.Y - V2.X * V0.Y) * den;
	Ret.X = 1.0f - Ret.Y - Ret.Z;

	if (Tolerance != 0.f && !ValidBaryCentric(Ret.X, Ret.Y, Ret.Z))
	{
		static const float NTolerance = 0.f - Tolerance;
		static const float PTolerance = 1.f + Tolerance;
		if (Ret.X > NTolerance && Ret.X < 0.f)
		{
			Ret.X = 0.f;
		}
		if (Ret.X < PTolerance && Ret.X > 1.f)
		{
			Ret.X = 1.f;
		}
		if (Ret.Y > NTolerance && Ret.Y < 0.f)
		{
			Ret.Y = 0.f;
		}
		if (Ret.Y < PTolerance && Ret.Y > 1.f)
		{
			Ret.Y = 1.f;
		}
		if (Ret.Z > NTolerance && Ret.Z < 0.f)
		{
			Ret.Z = 0.f;
		}
		if (Ret.Z < PTolerance && Ret.Z > 1.f)
		{
			Ret.Z = 1.f;
		}
	}
	return Ret;
}


bool AManualDetourNavMesh::ValidBaryCentric(float u, float v, float w)
{
	return !(u < 0.f || u > 1.f || v < 0.f || v > 1.f || w < 0.f || w > 1.f);
}

float AManualDetourNavMesh::GetRatioFromTriSegment(FVector P, FVector A, FVector B, float Tolerance)
{
	// Cross product determines colinearity
	if (FMath::Abs((B.X - A.X) * (P.Y - A.Y) - (P.X - A.X) * (B.Y - A.Y)) < Tolerance)
	{
		float DiffPA = FVector::Dist2D(P, A);
		float DiffBA = FVector::Dist2D(B, A);
		return DiffPA / DiffBA;	
	}
	else {
		return -1.f;
	}
}
