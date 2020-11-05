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
#include "DrawDebugHelpers.h"
#include "WorldMap.h"
#include "RegionDistribution.h"
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

			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge %d of poly %2d EdgeCode %4hu (%2hu)-(%2hu) :: %s - %s"), j, i, EdgeCode, Ind1, Ind2, *V1.ToCompactString(), *V2.ToCompactString());

			MoveToCentre(V1, PolyCentre,DebugThickness);
			MoveToCentre(V2, PolyCentre,DebugThickness);
			DrawDebugLine(GetWorld(), V1, V2, EdgeColor, true, 999.f, 0, DebugThickness);
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
	verts[Index * 3] = (unsigned short)(FMath::FloorToInt((TileParams.bmin[0] + V.X) * ncsinv));
	verts[Index * 3 + 1] = (unsigned short)(FMath::FloorToInt((V.Z - TileParams.bmin[1]) * chinv));
	verts[Index * 3 + 2] = (unsigned short)(FMath::FloorToInt((TileParams.bmin[2] + V.Y) * ncsinv));
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

	TSet<int32> TileTriangles;

#if WITH_EDITOR
	if (bDebugDraw)
	{
		DrawDebugBox(GetWorld(), FVector(Bounds.GetCenter(), 0.f), FVector(Bounds.GetExtent(), 1.f), FColor::Cyan, true, 999.f, 0, DebugThickness);
	}
#endif

	TArray<FVector2D> TmpVerts;
	TmpVerts.Reserve(TempReserveVerts);
	TArray<unsigned short> TmpPolys;
	TmpPolys.Reserve(2 * TempReservePolys * 6);
	FMemory::Memset(TmpPolys.GetData(), 0xff, sizeof(unsigned short) * TempReservePolys * 6 * 2);


	// Mapping from triangulation vert index to vert in this tile
	TMap<int32, FPCGTriPoint> InternalVerts;
	InternalVerts.Reserve(TempReserveVerts);

	// Mapping from triangulation half edge index to a derived vert in this tile
	TMap<int32, FPCGTriPoint> HalfEdgeVerts;
	HalfEdgeVerts.Reserve(TempReserveVerts);

	// First go through all the triangles in the mesh and find the points/tris/edges we're interested in for this tile
	int32 MaxPolyVerts = 3;

	int32 TmpCurrentPoly = 0;
	for (int32 i = 0; i < Triangulation.Triangles.Num() / 3; i++)
	{
		int32 VertCount = ProcessTriangle(Bounds, i, Triangulation, TmpVerts, TmpPolys, TmpCurrentPoly, InternalVerts, HalfEdgeVerts);
		if (VertCount > 0)
		{
			TileTriangles.Add(i);
			UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Triangle %d has %d verts"), i, VertCount);
		}
		MaxPolyVerts = FMath::Max(MaxPolyVerts, VertCount);
	}
//	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("MaxPolyVerts for tile %d, Corner Verts %d, EdgeVerts, %d, InternalVerts %d"), MaxPolyVerts, CornerVerts, EdgeVerts, InternalVerts.Num());

#if WITH_EDITOR
	if (bDebugDraw)
	{
		for (auto Tri : TileTriangles)
		{
			URegionDistribution::DrawTriangle(this, Tri, Triangulation, DebugThickness, Tri);
		}
	}
#endif WITH_EDITOR

	// Vertices per polygon
	TileParams.nvp = MaxPolyVerts;

	// Vertices per polygon
	TileParams.vertCount = 4;

	// Number of polys in the tile
	TileParams.polyCount = 1;

	// These will be the same apart from fixed interval vs floating point.
	unsigned short* verts = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * TileParams.vertCount * 3);

	SetDetourVector(verts, FVector(Bounds.Min.X, Bounds.Min.Y, GetHeightAt(Bounds.Min.X, Bounds.Min.Y)), 0, TileParams);
	SetDetourVector(verts, FVector(Bounds.Max.X, Bounds.Min.Y, GetHeightAt(Bounds.Max.X, Bounds.Min.Y)), 1, TileParams);
	SetDetourVector(verts, FVector(Bounds.Max.X, Bounds.Max.Y, GetHeightAt(Bounds.Max.X, Bounds.Max.Y)), 2, TileParams);
	SetDetourVector(verts, FVector(Bounds.Min.X, Bounds.Max.Y, GetHeightAt(Bounds.Min.X, Bounds.Max.Y)), 3, TileParams);

	// This is list of vert indices with additional edge flags. 0xFF means no vert index and solid border edge
	unsigned short* polys = (unsigned short*)FMemory::Malloc(sizeof(unsigned short) * TileParams.polyCount * TileParams.nvp * 2);
	memset(polys, 0xff, sizeof(unsigned short) * TileParams.polyCount * TileParams.nvp * 2);
	polys[0] = 0;
	polys[1] = 1;
	polys[2] = 2;
	polys[3] = 3;


	// Then go through the points for each triangle and snap them to the cell grid.
	// Have the lower triangle index "own" the vertex, so a neighbour with higher index can get the vert from there
	// There's the chance we need to clean up the input points for cases where points are too close to each other and need to be merged
	// It's a bit easier to determine if edges lie exactly along the boundary when they're expressed as ints/shorts so do that instead of working with floats
	// - except determining the intersection point between the edge and the tile bounds which might be better to do with floats (?)
	for (auto Tri : TileTriangles)
	{
		AddTriToPoly(Tri, TileParams, Triangulation, verts, polys);
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
}

int32 AManualDetourNavMesh::ProcessTriangle(const FBox2D& Bounds, int32 TriIndex, const FPCGDelaunayTriangulation &Triangulation,
		TArray<FVector2D> &TmpVerts, TArray<unsigned short> &TmpPolys, int32 &CurrentPoly, TMap<int32, FPCGTriPoint> &InternalVerts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts)
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
		return 0;
	}

	// Get the status of all the points in this triangle
	FPCGTriPoint Av = ProcessPoint(Ai, Bounds, Triangulation.Coords, TmpVerts, InternalVerts);
	FPCGTriPoint Bv = ProcessPoint(Bi, Bounds, Triangulation.Coords, TmpVerts, InternalVerts);
	FPCGTriPoint Cv = ProcessPoint(Ci, Bounds, Triangulation.Coords, TmpVerts, InternalVerts);

	// If all points are within the bounds, we can create the poly and get outta here
	if (Av.Key >= 0 && Bv.Key >= 0 && Cv.Key >= 0)
	{
		// TODO make poly
		return 3;
	}


	// If all the corners are within this triangle we can make a quad and be done. This
	// also prevents having to handle this case when walking along edges, which would be
	// quite awkward without any obvious starting point
	FPCGTriCorners Corners;

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
		return 4;
	}

	// Start testing the edges to see if there's intersections with the bounds and/or colinear segments with the bounds
	FPCGTriPoint EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC;

	ProcessEdge(EdgeAB, EdgeBA, 3*TriIndex, Av, Bv, Bounds, Triangulation, TmpVerts, TmpPolys, CurrentPoly, InternalVerts, HalfEdgeVerts);
	ProcessEdge(EdgeBC, EdgeCB, 3*TriIndex+1, Bv, Cv, Bounds, Triangulation, TmpVerts, TmpPolys, CurrentPoly, InternalVerts, HalfEdgeVerts);
	ProcessEdge(EdgeCA, EdgeAC, 3*TriIndex+2, Cv, Av, Bounds, Triangulation, TmpVerts, TmpPolys, CurrentPoly, InternalVerts, HalfEdgeVerts);

	LogTriPoly(TriIndex, EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC, Av, Bv, Cv, NE, SE, SW, NW);
	int32 Count = PointCount(EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC, Av, Bv, Cv, NE, SE, SW, NW);

	// If there's less than 3 valid points then we might have found two edge vertices with the third pointing externally
	// which is a valid triangle but not for this tile
	if (Count < 3)
	{
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

	TriPointArray PointsToPlace;
	if (Av.Key >= 0)
		PointsToPlace.Add(TriPointType(&Av, true));
	if (EdgeAB.Key >= 0)
		PointsToPlace.Add(TriPointType(&EdgeAB, false));
	if (EdgeBA.Key >= 0)
		PointsToPlace.Add(TriPointType(&EdgeBA, false));
	
	if (Bv.Key >= 0)
		PointsToPlace.Add(TriPointType(&Bv, true));
	if (EdgeBC.Key >= 0)
		PointsToPlace.Add(TriPointType(&EdgeBC, false));
	if (EdgeCB.Key >= 0)
		PointsToPlace.Add(TriPointType(&EdgeCB, false));

	if (Cv.Key >= 0)
		PointsToPlace.Add(TriPointType(&Cv, true));
	if (EdgeCA.Key >= 0)
		PointsToPlace.Add(TriPointType(&EdgeCA, false));
	if (EdgeAC.Key >= 0)
		PointsToPlace.Add(TriPointType(&EdgeAC, false));

	PlacePoints(CurrentPoly, TriIndex, PointsToPlace, TmpVerts, TmpPolys, InternalVerts, HalfEdgeVerts, Corners, Triangulation);
	CurrentPoly++;
	return Count;
}



bool AManualDetourNavMesh::PlacePoints(int32 PolyIndex, int32 TriIndex, TriPointArray &PointsToPlace, TArray<FVector2D>& TmpVerts, TArray<unsigned short>& TmpPolys, TMap<int32, FPCGTriPoint>& InternalVerts,
	TMap<int32, FPCGTriPoint>& HalfEdgeVerts, FPCGTriCorners &Corners, const FPCGDelaunayTriangulation& Triangulation)
{
	for (int32 i = 0; i < PointsToPlace.Num(); i++)
	{
		TriPointType& PreviousPoint = i == 0 ? PointsToPlace.Last() : PointsToPlace[i - 1];
		TriPointType& CurrentPoint = PointsToPlace[i];

		// if current point is a half edge and previous point is a half edge, and the edge doesn't match, pull in corners
		if (!PreviousPoint.Value && !CurrentPoint.Value && CurrentPoint.Key->Value != PreviousPoint.Key->Value)
		{
			// add needed corner(s)
			PlaceCorners(CurrentPoint, PreviousPoint, PolyIndex, TriIndex, PointsToPlace, TmpVerts, TmpPolys, InternalVerts, HalfEdgeVerts, Corners, Triangulation);
		}

		// need the poly index so we can place in the right spot
		// Add vertex to tmppolys

		// need the triangulation so we can connect to adjacent polys.
		// Use the triangulation's numbering since we won't know the corrected numbering until everything has been placed
			// TODO actually we could do reciprocal updates as we go?
		// Add adjacency to tmppolys

	}
}

bool AManualDetourNavMesh::PlaceCorners(TriPointType& CurrentPoint, TriPointType& PreviousPoint, int32 PolyIndex, int32 TriIndex, TriPointArray& PointsToPlace, TArray<FVector2D>& TmpVerts, TArray<unsigned short>& TmpPolys, TMap<int32, FPCGTriPoint>& InternalVerts,
	TMap<int32, FPCGTriPoint>& HalfEdgeVerts, FPCGTriCorners& Corners, const FPCGDelaunayTriangulation& Triangulation)
{
	return true;
}

FPCGTriPoint AManualDetourNavMesh::ProcessPoint(int32 PointIndex, const FBox2D& Bounds, const TArray<FVector2D>& Coords, TArray<FVector2D>& TmpVerts, TMap<int32, FPCGTriPoint>& InternalVerts)
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
	const FVector2D& TestPoint = Coords[PointIndex];
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

		if (TP.Value == EPCGTriPoint::E_External)
		{
			UE_LOG(LogManualNavMesh, Warning, TEXT("point %d (%s) lies within the border but wasn't assigned an internal code"), PointIndex, *TestPoint.ToString());
		}

		InternalVerts.Add(PointIndex, TP);
		return TP;
	}

	// Point is not within the bounds of this tile
	return FPCGTriPoint(-1, EPCGTriPoint::E_External);
}

void AManualDetourNavMesh::ProcessEdge(FPCGTriPoint& Edge1, FPCGTriPoint& Edge2, int32 EdgeIndex, const FPCGTriPoint& A, const FPCGTriPoint& B, const FBox2D& Bounds, const FPCGDelaunayTriangulation& Triangulation,
	TArray<FVector2D>& TmpVerts, TArray<unsigned short>& TmpPolys, int32& CurrentPoly, TMap<int32, FPCGTriPoint>& InternalVerts, TMap<int32, FPCGTriPoint>& HalfEdgeVerts)
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
	Edge1.Key = -1;
	Edge2.Key = -1;
	Edge1.Value = EPCGTriPoint::E_External;
	Edge2.Value = EPCGTriPoint::E_External;

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
		Edge1.Key = E1->Key;
		Edge1.Value = E1->Value;
	}
	int32 Edge2Index = Triangulation.HalfEdges[EdgeIndex] >= 0 ? Triangulation.HalfEdges[EdgeIndex] : -1;
	if (Edge2Index < 0)
	{
		FPCGTriPoint* E2 = HalfEdgeVerts.Find(Edge2Index);
		if (E2)
		{
			Edge2.Key = E2->Key;
			Edge2.Value = E2->Value;
		}
	}

	// Edge was already processed by our neighbour
	// TODO add an unprocessed option to the ENUM so we can track edges that didn't match
	// as well and prevent double-testing everything that isn't relevant
	if (Edge1.Key >= 0 || Edge2.Key >= 0)
	{
		return;
	}

	// We probably want to test colinear cases next because the intersection tests will give positive results but they
	// will be wrong if the segment is colinear
	int32 VertAi = Triangulation.Triangles[EdgeIndex];
	int32 VertBi = Triangulation.Triangles[Triangulation.NextHalfedge(EdgeIndex)];

	FVector2D VertA = Triangulation.Coords[VertAi];
	FVector2D VertB = Triangulation.Coords[VertBi];

	ColinearEdgeCheck(Edge1, EdgeIndex, A, B, VertA, VertB, Bounds, Triangulation, TmpVerts, HalfEdgeVerts);
	ColinearEdgeCheck(Edge2, Edge2Index, B, A, VertB, VertA, Bounds, Triangulation, TmpVerts, HalfEdgeVerts);

	// Edge was colinear, we are done 
	if (Edge1.Key >= 0 || Edge2.Key >= 0)
	{
		return;
	}

	// Intersection test. Only return positive result when A is external and B is internal, or return the
	// closer result to A if both are external.
	IntersectingEdgeCheck(Edge1, EdgeIndex, A, B, VertA, VertB, Bounds, Triangulation, TmpVerts, HalfEdgeVerts);
	IntersectingEdgeCheck(Edge2, Edge2Index, B, A, VertB, VertA, Bounds, Triangulation, TmpVerts, HalfEdgeVerts);

}

void AManualDetourNavMesh::ColinearEdgeCheck(FPCGTriPoint& Edge, int32 EdgeIndex, const FPCGTriPoint &PA, const FPCGTriPoint &PB, FVector2D& VA, FVector2D &VB, const FBox2D& Bounds, const FPCGDelaunayTriangulation& Triangulation,
	TArray<FVector2D>& TmpVerts, TMap<int32, FPCGTriPoint>& HalfEdgeVerts)
{

	if (VA.X == VB.X)
	{
		// AB Colinear with East Boundary
		if (VA.X == Bounds.Max.X)
		{
			if (VA.Y < Bounds.Min.Y && VB.Y > Bounds.Min.Y)
			{
				// A is South of the East boundary and B is either on or North of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Max.X, Bounds.Min.Y), Edge,EdgeIndex, EPCGTriPoint::E_SouthEast, TmpVerts, HalfEdgeVerts);
			}
			else if (VA.Y > Bounds.Max.Y && VB.Y < Bounds.Max.Y)
			{
				// A is North of the East boundary and B is either on or South of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Max.X, Bounds.Min.Y), Edge,EdgeIndex, EPCGTriPoint::E_NorthEast, TmpVerts, HalfEdgeVerts);
			}
		}
		// AB Colinear with West Boundary
		else if (VA.X == Bounds.Min.X)
		{
			if (VA.Y < Bounds.Min.Y && VB.Y > Bounds.Min.Y)
			{
				// A is South of the West boundary and B is either on or North of it
				AddHalfEdgeTempVert(Bounds.Min, Edge,EdgeIndex, EPCGTriPoint::E_SouthWest, TmpVerts, HalfEdgeVerts);
			}
			else if (VA.Y > Bounds.Max.Y && VB.Y < Bounds.Max.Y)
			{
				// A is North of the West boundary and B is either on or South of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Min.X, Bounds.Max.Y), Edge,EdgeIndex, EPCGTriPoint::E_NorthWest, TmpVerts, HalfEdgeVerts);
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
				AddHalfEdgeTempVert(FVector2D(Bounds.Min.X, Bounds.Max.Y), Edge,EdgeIndex, EPCGTriPoint::E_NorthWest, TmpVerts, HalfEdgeVerts);
			}
			else if (VA.X > Bounds.Max.X && VB.X < Bounds.Max.X)
			{
				// A is East of the North boundary and B is either on or West of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Max.X, Bounds.Min.Y), Edge,EdgeIndex, EPCGTriPoint::E_NorthEast, TmpVerts, HalfEdgeVerts);
			}
		}
		// AB Colinear with South Boundary
		else if (VA.Y == Bounds.Min.Y)
		{
			if (VA.X < Bounds.Min.X && VB.X > Bounds.Min.X)
			{
				// A is West of the South boundary and B is either on or East of it
				AddHalfEdgeTempVert(Bounds.Min, Edge,EdgeIndex, EPCGTriPoint::E_SouthWest, TmpVerts, HalfEdgeVerts);
			}
			else if (VA.X > Bounds.Max.X && VB.X < Bounds.Max.X)
			{
				// A is East of the South boundary and B is either on or West of it
				AddHalfEdgeTempVert(FVector2D(Bounds.Max.X, Bounds.Min.Y), Edge,EdgeIndex, EPCGTriPoint::E_SouthEast, TmpVerts, HalfEdgeVerts);
			}
		}
	}

}
void AManualDetourNavMesh::IntersectingEdgeCheck(FPCGTriPoint& Edge, int32 EdgeIndex, const FPCGTriPoint& PA, const FPCGTriPoint& PB, FVector2D& VA, FVector2D& VB, const FBox2D& Bounds, const FPCGDelaunayTriangulation& Triangulation,
	TArray<FVector2D>& TmpVerts, TMap<int32, FPCGTriPoint>& HalfEdgeVerts)
{
	// TODO the intersection test should also give us the height for the intersect point

	// Intersection test. Only return positive result when A is external and B is internal, or return the
	// closer result to A if both are external.
	if (PA.Value != EPCGTriPoint::E_External)
	{
		return;
	}

	FVector B1 = FVector(Bounds.Min.X, Bounds.Max.Y, 0.f);
	FVector B2 = FVector(Bounds.Max.X, Bounds.Min.Y, 0.f);

	// Note that we don't want cases where B is on the edge, only actual internal points give intersections 
	if (PA.Value == EPCGTriPoint::E_External && PB.Value == EPCGTriPoint::E_Internal)
	{

		// TODO this might need to be corrected to the corner in the unlikely event the segment intersects with the corner
		DrawDebugLine(GetWorld(), FVector(VA, 0.f), FVector(VB, 0.f), FColor::Purple, true, 999.f, 0, 2.f);

		DrawDebugLine(GetWorld(), FVector(Bounds.Min, 0.f), B1, FColor::Red, true, 999.f, 0, 2.f);
		DrawDebugLine(GetWorld(), FVector(Bounds.Max, 0.f), B1, FColor::Orange, true, 999.f, 0, 2.f);
		DrawDebugLine(GetWorld(), FVector(Bounds.Max, 0.f), B2, FColor::Yellow, true, 999.f, 0, 2.f);
		DrawDebugLine(GetWorld(), FVector(Bounds.Min, 0.f), B2, FColor::Green, true, 999.f, 0, 2.f);

		FVector IntersectPoint;
		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B1, IntersectPoint))
		{
			AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, EdgeIndex, EPCGTriPoint::E_West, TmpVerts, HalfEdgeVerts);
			return;
		}
		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B1, IntersectPoint))
		{
			AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, EdgeIndex, EPCGTriPoint::E_North, TmpVerts, HalfEdgeVerts);
			return;
		}
		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B2, IntersectPoint))
		{
			AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, EdgeIndex, EPCGTriPoint::E_East, TmpVerts, HalfEdgeVerts);
			return;
		}
		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B2, IntersectPoint))
		{
			AddHalfEdgeTempVert(FVector2D(IntersectPoint), Edge, EdgeIndex, EPCGTriPoint::E_South, TmpVerts, HalfEdgeVerts);
			return;
		}

	}

	// Will either be 2 intersections and pick the closest, or none at all.
	if (PA.Value == EPCGTriPoint::E_External && PB.Value == EPCGTriPoint::E_External)
	{
		FVector IntersectPoint1;
		FVector IntersectPoint2;
		EPCGTriPoint IP1Type = EPCGTriPoint::E_External;
		EPCGTriPoint IP2Type = EPCGTriPoint::E_External;

		FVector& CurrentPoint = IntersectPoint1;

		bool bFoundP1 = false;
		bool bFoundP2 = false;

		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B1, CurrentPoint))
		{
			bFoundP1 = true;
			CurrentPoint = IntersectPoint2;
			IP1Type = EPCGTriPoint::E_West;
		}

		if (FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B1, CurrentPoint))
		{

			if (bFoundP1)
			{
				bFoundP2 = true;
				IP2Type = EPCGTriPoint::E_North;
			}
			else {
				bFoundP1 = true;
				CurrentPoint = IntersectPoint2;
				IP1Type = EPCGTriPoint::E_North;
			}

		}

		if (!bFoundP2 && FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Max, 0.f), B2, CurrentPoint))
		{
			if (bFoundP1)
			{
				bFoundP2 = true;
				IP2Type = EPCGTriPoint::E_East;
			}
			else {
				bFoundP1 = true;
				CurrentPoint = IntersectPoint2;
				IP1Type = EPCGTriPoint::E_East;
			}
		}

		if (!bFoundP2 && FMath::SegmentIntersection2D(FVector(VA, 0.f), FVector(VB, 0.f), FVector(Bounds.Min, 0.f), B2, CurrentPoint))
		{
			if (bFoundP1)
			{
				bFoundP2 = true;
				IP2Type = EPCGTriPoint::E_South;
			}
			else {
				bFoundP1 = true;
				CurrentPoint = IntersectPoint2;
				IP1Type = EPCGTriPoint::E_South;
			}
		}

		if (bFoundP2)
		{
			FVector2D P12D = FVector2D(IntersectPoint1);
			FVector2D P22D = FVector2D(IntersectPoint2);

			float DistAtoP1Sq = FVector2D::DistSquared(VA, P12D);
			float DistAtoP2Sq = FVector2D::DistSquared(VA, P22D);

			if (DistAtoP1Sq > DistAtoP2Sq)
			{
				AddHalfEdgeTempVert(P22D, Edge, EdgeIndex, IP2Type, TmpVerts, HalfEdgeVerts);
			}
			else {
				AddHalfEdgeTempVert(P12D, Edge, EdgeIndex, IP2Type, TmpVerts, HalfEdgeVerts);
			}
		}
	}
}

void AManualDetourNavMesh::AddHalfEdgeTempVert(FVector2D Location, FPCGTriPoint& Point, int32 EdgeIndex, EPCGTriPoint Type, TArray<FVector2D> &Verts, TMap<int32, FPCGTriPoint> &HalfEdgeVerts)
{
	Point.Key = Verts.Num();
	Point.Value = Type;
	Verts.Add(Location);
	HalfEdgeVerts.Add(EdgeIndex, Point);
}

bool AManualDetourNavMesh::IsInternalSegment(const FPCGTriPoint& A, const FPCGTriPoint& B)
{
	/*
		Check each boundary
	*/
	if ((A.Value == EPCGTriPoint::E_East || A.Value == EPCGTriPoint::E_NorthEast || A.Value == EPCGTriPoint::E_SouthEast) &&
		(B.Value == EPCGTriPoint::E_East || B.Value == EPCGTriPoint::E_NorthEast || B.Value == EPCGTriPoint::E_SouthEast))
	{
		return true;
	}

	if ((A.Value == EPCGTriPoint::E_West || A.Value == EPCGTriPoint::E_NorthWest || A.Value == EPCGTriPoint::E_SouthWest) &&
		(B.Value == EPCGTriPoint::E_West || B.Value == EPCGTriPoint::E_NorthWest || B.Value == EPCGTriPoint::E_SouthWest))
	{
		return true;
	}

	if ((A.Value == EPCGTriPoint::E_South || A.Value == EPCGTriPoint::E_SouthEast || A.Value == EPCGTriPoint::E_SouthWest) &&
		(B.Value == EPCGTriPoint::E_South || B.Value == EPCGTriPoint::E_SouthEast || B.Value == EPCGTriPoint::E_SouthWest))
	{
		return true;
	}

	if ((A.Value == EPCGTriPoint::E_North || A.Value == EPCGTriPoint::E_NorthEast || A.Value == EPCGTriPoint::E_NorthWest) &&
		(B.Value == EPCGTriPoint::E_South || B.Value == EPCGTriPoint::E_NorthEast || B.Value == EPCGTriPoint::E_NorthWest))
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
	FString Ret = FString::FromInt(Key);
	Ret.AppendChar(' ');

	switch (Value)
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
	if (Point.Key >= 0)
	{
		if (Point.Value == EPCGTriPoint::E_NorthEast)
			Corners.NE = false;
		if (Point.Value == EPCGTriPoint::E_SouthEast)
			Corners.SE = false;
		if (Point.Value == EPCGTriPoint::E_SouthWest)
			Corners.SW = false;
		if (Point.Value == EPCGTriPoint::E_NorthWest)
			Corners.NW = false;
		return 1;
	}
	return 0;
}

void AManualDetourNavMesh::LogTriPoly(int32 Index, FPCGTriPoint &EdgeAB, FPCGTriPoint &EdgeBA,FPCGTriPoint &EdgeBC,FPCGTriPoint &EdgeCB,FPCGTriPoint &EdgeCA,FPCGTriPoint &EdgeAC, FPCGTriPoint &A, FPCGTriPoint &B, FPCGTriPoint &C, FPCGTriCorners& Corners)
{
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Processing triangle %d, PointCount: %d"), Index, PointCount(EdgeAB, EdgeBA, EdgeBC, EdgeCB, EdgeCA, EdgeAC, A, B, C, Corners));
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Point A : %s :: Point B : %s :: Point C %s"), *A.ToString(), *B.ToString(), *C.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge AB : %s :: Edge BA : %s"), *EdgeAB.ToString(), *EdgeBA.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge BC : %s :: Edge CB : %s"), *EdgeBC.ToString(), *EdgeCB.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Edge CA : %s :: Edge AC : %s"), *EdgeCA.ToString(), *EdgeAC.ToString());
	UE_LOG(LogManualNavMesh, VeryVerbose, TEXT("Corners: %s"), *CornerString(NE, SE, SW, NW));
}
