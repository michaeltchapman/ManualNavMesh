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
#include "ManualNavmesh.h"

DEFINE_LOG_CATEGORY(LogManualNavMesh)

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

	for (int y = 0; y < TileCount; ++y)
	{
		for (int x = 0; x < TileCount; ++x)
		{
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

	MakeGridTile(TileParams, tx, ty);

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

FVector AManualDetourNavMesh::GetMin() const
{
	return Min;
}

FVector AManualDetourNavMesh::GetMax() const
{
	return Max;
}