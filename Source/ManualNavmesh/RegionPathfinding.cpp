#include "RegionPathfinding.h"
#include "RegionDistribution.h"

DEFINE_LOG_CATEGORY(LogUPCGTRegionPathfinding);

bool FPCGPathfindingFlags::CanTraverse(int32 Region)
{
	return Flags[Region] > 0;
}

int32 FPCGTPAPath::GetCurrentTriangle() const
{
	return CurrentTriangle;
}

int32 FPCGTPAPath::GetCurrentEdge() const
{
	return CurrentEdge;
}

void FPCGTPAPath::DebugDraw(FVector2D Offset/* = FVector2D::ZeroVector*/) const
{
	Funnel.DebugDraw(Offset);
	//Triangulation->DrawTriangle(Triangulation->DebugWorld, CurrentTriangle, 1.f, FColor::Yellow);
}

void FPCGPathFunnel::DebugDraw(FVector2D Offset) const
{
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* CurrentNode = Funnel.GetHead();
	while (CurrentNode && CurrentNode->GetNextNode() != nullptr)
	{
		//DrawDebugLine(Triangulation->DebugWorld, FVector(CurrentNode->GetValue()+Offset, 1.f), FVector(CurrentNode->GetValue()+Offset, 1.f), FColor::Blue);
		CurrentNode = CurrentNode->GetNextNode();
	}

	CurrentNode = Path.GetHead();
	while (CurrentNode && CurrentNode->GetNextNode() != nullptr)
	{
		//DrawDebugLine(Triangulation->DebugWorld, FVector(CurrentNode->GetValue()+Offset, 1.f), FVector(CurrentNode->GetValue()+Offset, 1.f), FColor::Cyan);
		CurrentNode = CurrentNode->GetNextNode();
	}

	if (Apex)
	{
		//DrawDebugSphere(Triangulation->DebugWorld, FVector(Apex->GetValue()+Offset, 1.f), 5.f, 4, FColor::Emerald, true);
	}
}

void FPCGTPAPathFinder::DebugDraw(FVector2D Offset) const
{
	for (auto& Path : OpenSet)
	{
		Path.DebugDraw(Offset);
	}
}

void FPCGTPAPath::DebugLog() const
{
	//UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Path: Current Triangle %d, Current Edge %d; AlreadyBuiltLength %s | ShortLengthApexToEdge %s | LongLengthApexToEdge %s | DistanceFromGoal %s "),
//		CurrentTriangle, CurrentEdge, *FString::SanitizeFloat(AlreadyBuiltPathLength), *FString::SanitizeFloat(LengthOfShortestPathFromApexToEdge), *FString::SanitizeFloat(LengthOfLongestPathFromApexToEdge), *FString::SanitizeFloat(DistanceFromClosestGoalPoint));

//	UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Path Funnel:"));
//	Funnel.DebugLog();

	UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("%s"), *ToString());
}

void FPCGPathFunnel::DebugLog() const
{
	//UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Funnel: Funnel Num: %d, Path Num %d, Apex: %s"),
//		Funnel.Num(), Path.Num(), Apex == nullptr ? *FString(TEXT("null")) : *Apex->GetValue().ToString());
	UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("%s"), *ToString());
}

void FPCGTPAPathFinder::DebugLog() const
{
	UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("PathFinder: OpenSet Num: %d"), OpenSet.Num());
	UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("PathFinder: Paths:"));

	TDoubleLinkedList<FPCGTPAPath>::TDoubleLinkedListNode* Node = OpenSet.GetHead();
	while (Node)
	{
		Node->GetValue().DebugLog();
		Node = Node->GetNextNode();
	}
}

FString FPCGTPAPath::ToString() const
{
	FString Ret = FString(FString::Printf(TEXT("Path: Current Triangle %d, Current Edge %d; MinTotalCost %s | ShortLengthApexToEdge %s | LongLengthApexToEdge %s | DistanceFromGoal %s | Funnel: %s"),
		CurrentTriangle, CurrentEdge, *FString::SanitizeFloat(MinimalTotalCost()), *FString::SanitizeFloat(LengthOfShortestPathFromApexToEdge), *FString::SanitizeFloat(LengthOfLongestPathFromApexToEdge), *FString::SanitizeFloat(DistanceFromClosestGoalPoint)
		, *Funnel.ToString()));
	return Ret;
}

FString FPCGPathFunnel::ToString() const
{
	FString Ret = FString(FString::Printf(TEXT("::Funnel:: Apex: %s, Path length %d, Funnel: "), Apex == nullptr ? *FString(TEXT("null")) : *Apex->GetValue().ToString(), Path.Num()));

	auto CurrentNode = Funnel.GetHead();
	while (CurrentNode != nullptr)
	{
		auto& Vec = CurrentNode->GetValue();
		bool bAdded = false;
		for (int32 i = 0; i < Triangulation->Coords.Num(); i++)
		{
			if (Triangulation->Coords[i] == Vec)
			{
				Ret.AppendInt(i);
				Ret.AppendChar(' ');
				bAdded = true;
				break;
			}
		}

		if (!bAdded)
		{
			Ret.Append(Vec.ToString());
			Ret.AppendChar(' ');
		}

		CurrentNode = CurrentNode->GetNextNode();
	}

	return Ret;
}

FPCGPathFunnel::FPCGPathFunnel(FVector2D StartPoint, FPCGDelaunayTriangulation *inTriangulation)
{
	Triangulation = inTriangulation;
	Funnel.AddHead(StartPoint);
	Apex = Funnel.GetHead();
	Path.AddHead(StartPoint);
}

FPCGPathFunnel::FPCGPathFunnel(const FPCGPathFunnel &Other)
{
	FPCGTPAPathFinder::CloneDoubleLL(Other.Funnel, Funnel);
	Apex = Funnel.FindNode(Other.Apex->GetValue());
	FPCGTPAPathFinder::CloneDoubleLL(Other.Path, Path);
	Triangulation = Other.Triangulation;
}

FPCGPathFunnel& FPCGPathFunnel::operator=(const FPCGPathFunnel& Other)
{
	FPCGTPAPathFinder::CloneDoubleLL(Other.Funnel, Funnel);
	Apex = Funnel.FindNode(Other.Apex->GetValue());
	FPCGTPAPathFinder::CloneDoubleLL(Other.Path, Path);
	Triangulation = Other.Triangulation;
	return *this;
}

void FPCGPathFunnel::StepOver(int32 Edge)
{
	if (Edge < 0)
	{
		UE_LOG(LogUPCGTRegionPathfinding, Error, TEXT("Invalid edge provided to StepOver"));
		return;
	}

	FVector2D EdgeA = Triangulation->Coords[Triangulation->HalfEdgeOrigin(Edge)];
	FVector2D EdgeB = Triangulation->Coords[Triangulation->HalfEdgeDest(Edge)];

	EPCGFunnelSide CommonSide =DetermineSideSharedBy(Edge, Funnel);
	FVector2D LeftEnd = Funnel.GetHead()->GetValue();
	FVector2D RightEnd = Funnel.GetTail()->GetValue();

	if (CommonSide == EPCGFunnelSide::E_Both)
	{
		UE_LOG(LogUPCGTRegionPathfinding, Error, TEXT("Edge endpoints are identical with funnel endpoints"));
		return;
	}
	
	if (CommonSide == EPCGFunnelSide::E_Left)
	{
		FVector2D PointToAdd = LeftEnd.Equals(EdgeA) ? EdgeB : EdgeA;
		AddToRightSideOfFunnel(PointToAdd);
	}
	else if (CommonSide == EPCGFunnelSide::E_Right)
	{
		FVector2D PointToAdd = RightEnd.Equals(EdgeA) ? EdgeB : EdgeA;
		AddToLeftSideOfFunnel(PointToAdd);
	}
	//(CommonSide == EPCGFunnelSide::E_None)
	else
	{
		if (Funnel.Num() == 1)
		{
			InitFunnel(Edge); // this is the first edge
		} else
		{
			UE_LOG(LogUPCGTRegionPathfinding, Error, TEXT("Funnel end and new edge do not share any vertex"));
			return;
		}
	}
}

void FPCGPathFunnel::FinalisePath(FVector2D Goal)
{
	AddToRightSideOfFunnel(Goal);
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Node = Apex;
	while (!Node->GetValue().Equals(Goal))
	{
		Node = Node->GetNextNode();
		Path.AddTail(Node->GetValue());
	}
}

void FPCGPathFunnel::InitFunnel(int32 FirstEdge)
{
	if (!Triangulation->PointLiesOnEdge(FirstEdge, Apex->GetValue()))
	{
		FVector2D FirstEdgeA = Triangulation->Coords[Triangulation->HalfEdgeOrigin(FirstEdge)];
		FVector2D FirstEdgeB = Triangulation->Coords[Triangulation->HalfEdgeDest(FirstEdge)];
		FVector2D ApexToEdgeA = FirstEdgeA - Apex->GetValue();
		FVector2D ApexToEdgeB = FirstEdgeB - Apex->GetValue();

		if (Triangulation->IsInCounterClockwiseDirectionFrom(ApexToEdgeA, ApexToEdgeB))
		{
			Funnel.AddHead(FirstEdgeA);
			Funnel.AddTail(FirstEdgeB);
		}
		if (Triangulation->IsInClockwiseDirectionFrom(ApexToEdgeA, ApexToEdgeB))
		{
			Funnel.AddHead(FirstEdgeB);
			Funnel.AddTail(FirstEdgeA);
		}
	}
}

EPCGFunnelSide FPCGPathFunnel::DetermineSideSharedBy(int32 Edge, TDoubleLinkedList<FVector2D> &InFunnel)
{
	const FVector2D LeftEnd = InFunnel.GetHead()->GetValue();
	const FVector2D RightEnd = InFunnel.GetTail()->GetValue();
	const FVector2D &EdgeA = Triangulation->Coords[Triangulation->HalfEdgeOrigin(Edge)];
	const FVector2D &EdgeB = Triangulation->Coords[Triangulation->HalfEdgeDest(Edge)];
	const bool bLeftEndEqualsA = LeftEnd.Equals(EdgeA);
	const bool bLeftEndEqualsB = LeftEnd.Equals(EdgeB);
	const bool bRightEndEqualsA = RightEnd.Equals(EdgeA);
	const bool bRightEndEqualsB = RightEnd.Equals(EdgeB);

	if ((bLeftEndEqualsA && bRightEndEqualsA) || (bLeftEndEqualsB && bRightEndEqualsA))
	{
		return EPCGFunnelSide::E_Both;
	}
	if (bLeftEndEqualsA || bLeftEndEqualsB)
	{
		return EPCGFunnelSide::E_Left;
	}
	if (bRightEndEqualsA || bRightEndEqualsB)
	{
		return EPCGFunnelSide::E_Right;
	}
	return EPCGFunnelSide::E_None;	
}

void FPCGPathFunnel::AddToRightSideOfFunnel(FVector2D Point)
{
	Funnel.AddTail(Point);
	FVector2D RightEndPoint = Point;
	bool bPopped = true;
	while (bPopped && (Funnel.GetTail()->GetPrevNode() != Apex))
	{
		bPopped = false;
		TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode *SecondItemFromRight = Funnel.GetTail()->GetPrevNode();
		TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode *ThirdItemFromRight = SecondItemFromRight->GetPrevNode();
		FVector2D BackOne = SecondItemFromRight->GetValue() - RightEndPoint;
		FVector2D BackTwo = ThirdItemFromRight->GetValue() - RightEndPoint;
		if (Triangulation->IsInClockwiseDirectionFrom(BackTwo, BackOne))
		{
			Funnel.RemoveNode(SecondItemFromRight);
			bPopped = true;
		}
	}
	
	bPopped = true;
	while (bPopped && (Funnel.GetHead() != Apex))
	{
		bPopped = false;
		const FVector2D ApexToRightEnd = Funnel.GetTail()->GetValue() - Apex->GetValue();
		const FVector2D ApexToOneLeft = Apex->GetPrevNode()->GetValue() - Apex->GetValue();
		if (Triangulation->IsInCounterClockwiseDirectionFrom(ApexToRightEnd, ApexToOneLeft))
		{
			Path.AddTail(Apex->GetPrevNode()->GetValue());
			Funnel.RemoveNode(Apex);
			Apex = Funnel.GetTail()->GetPrevNode();
			bPopped = true;
		}
	}
}

void FPCGPathFunnel::AddToLeftSideOfFunnel(FVector2D Point)
{
		Funnel.AddHead(Point);
    	FVector2D LeftEndPoint = Point;
    	bool bPopped = true;
    	while (bPopped && (Funnel.GetHead()->GetNextNode() != Apex))
    	{
    		bPopped = false;
    		TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode *SecondItemFromLeft = Funnel.GetHead()->GetNextNode();
    		TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode *ThirdItemFromLeft = SecondItemFromLeft->GetNextNode();
    		const FVector2D BackOne = SecondItemFromLeft->GetValue() - LeftEndPoint;
    		const FVector2D BackTwo = ThirdItemFromLeft->GetValue() - LeftEndPoint;
    		if (Triangulation->IsInCounterClockwiseDirectionFrom(BackTwo, BackOne))
    		{
    			Funnel.RemoveNode(SecondItemFromLeft);
    			bPopped = true;
    		}
    	}
    	
    	bPopped = true;
    	while (bPopped && (Funnel.GetTail() != Apex))
    	{
    		bPopped = false;
    		const FVector2D ApexToLeftEnd = Funnel.GetHead()->GetValue() - Apex->GetValue();
    		const FVector2D ApexToOneRight = Apex->GetNextNode()->GetValue() - Apex->GetValue();
    		if (Triangulation->IsInClockwiseDirectionFrom(ApexToLeftEnd, ApexToOneRight))
    		{
    			Path.AddTail(Apex->GetNextNode()->GetValue());
    			Funnel.RemoveNode(Apex);
    			Apex = Funnel.GetHead()->GetNextNode();
    			bPopped = true;
    		}
    	}
}

float FPCGTPAPath::ShortestPathToEdgeLength() const
{
	return AlreadyBuiltPathLength + LengthOfShortestPathFromApexToEdge;
}

float FPCGTPAPath::LongestPathToEdgeLength() const
{
	return AlreadyBuiltPathLength + LengthOfLongestPathFromApexToEdge;
}

float FPCGTPAPath::MinimalTotalCost() const
{
	return ShortestPathToEdgeLength() + DistanceFromClosestGoalPoint;
}

FPCGTPAPath FPCGTPAPath::BuildPartialPathTo(int32 Neighbour, FVector2D Goal)
{
	bool bIsAdjacent = false;
	TriangleNeighbours Neighbours = Triangulation->GetTriangleNeighbours(CurrentTriangle);
	for (auto Triangle : Neighbours)
	{
		if (!FPCGTPAPathFinder::CanTraverse(*Triangulation, Neighbour))
		{
			continue;
		}

		if (Triangle == Neighbour)
		{
			bIsAdjacent = true;
		}
	}
	if (!bIsAdjacent)
	{
		UE_LOG(LogUPCGTRegionPathfinding, Error, TEXT("Edge endpoints are identical with funnel endpoints"));
		return FPCGTPAPath();
	}

	FPCGTPAPath NewPath = this->Clone();
	NewPath.StepTo(Neighbour, Goal);
	return NewPath;
}

void FPCGTPAPath::BuildCompletePathTo(FVector2D Goal, TDoubleLinkedList<FVector2D> &OutPath)
{
	if (Triangulation->IsInTriangle(Goal, CurrentTriangle))
	{
		FPCGPathFunnel FunnelCopy = FPCGPathFunnel(Funnel);
		FunnelCopy.FinalisePath(Goal);
		FPCGTPAPathFinder::CloneDoubleLL(FunnelCopy.Path, OutPath);
	}
}

void FPCGTPAPath::UpdateEstimationToClosestGoalPoint(FVector2D Goal)
{
	bool bShouldIncludeGoalsInCurrentTriangle = !bReachedPathsBuilt ? true : false;
	if (CurrentEdge >= 0)
	{
		DistanceFromClosestGoalPoint = MinimalDistanceBetween(CurrentEdge, Goal, bShouldIncludeGoalsInCurrentTriangle, CurrentTriangle);
	} else
	{
		DistanceFromClosestGoalPoint = MinimalDistanceBetween(Funnel.Apex->GetValue(), Goal, bShouldIncludeGoalsInCurrentTriangle, CurrentTriangle);
	}
}

void FPCGTPAPath::StepTo(int32 TargetTriangle, FVector2D Goal)
{
	CurrentEdge = Triangulation->GetCommonEdge(TargetTriangle, CurrentTriangle);
	CurrentTriangle = TargetTriangle;

	UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Stepping over edge %d, tri %d"), CurrentEdge, CurrentTriangle);

	Funnel.StepOver(CurrentEdge);
	AlreadyBuiltPathLength = LengthOfBuiltPathInFunnel(Funnel.Path);
	LengthOfShortestPathFromApexToEdge = GetLengthOfShortestPathFromApexToEdge(CurrentEdge, Funnel.Apex);
	LengthOfLongestPathFromApexToEdge = GetLengthOfLongestPathFromApexToEdge(Funnel.Apex);
	bReachedPathsBuilt = false;
	UpdateEstimationToClosestGoalPoint(Goal);
}

void FPCGTPAPath::PathToArray(TArray<FVector2D>& OutArray)
{
	OutArray.Empty();
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Node = Funnel.Path.GetHead();
	while (Node != nullptr)
	{
		OutArray.Add(Node->GetValue());
		Node = Node->GetNextNode();
	}
}

FPCGTPAPath FPCGTPAPath::Clone()
{
	return FPCGTPAPath(*this);
}


// these are static in the reference but we need the triangulation ptr to do almost anything 
float FPCGTPAPath::GetLengthOfShortestPathFromApexToEdge(int32 Edge, const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Apex)
{
	float Length = 0.f;
	if ((Apex->GetNextNode() != nullptr && Apex->GetPrevNode() != nullptr))
	{
		FVector2D ApexPoint = Apex->GetValue();
		FVector2D ClosestPointOfEdgeToApex = FMath::ClosestPointOnSegment2D(ApexPoint, Triangulation->Coords[Triangulation->HalfEdgeOrigin(Edge)], Triangulation->Coords[Triangulation->HalfEdgeDest(Edge)]);
		FVector2D ApexToLeftOne = Apex->GetPrevNode()->GetValue() - ApexPoint;
		FVector2D ApexToRightOne = Apex->GetNextNode()->GetValue() - ApexPoint;
		FVector2D ApexToClosestPointOnEdge = ClosestPointOfEdgeToApex - ApexPoint;
		if (Triangulation->IsInCounterClockwiseDirectionFrom(ApexToLeftOne, ApexToClosestPointOnEdge))
		{
			if (Triangulation->IsInClockwiseDirectionFrom(ApexToRightOne, ApexToClosestPointOnEdge))
			{
				Length = FVector2D::Distance(ApexPoint, ClosestPointOfEdgeToApex);
			}
			else {
				Length = WalkOnRightSideOfFunnelUntilClosestPointBecomesVisible(Apex, Edge);
			}
		}
		else {
			Length = WalkOnLeftSideOfFunnelUntilClosestPointBecomesVisible(Apex, Edge);
		}
	}
	return Length;
}

float FPCGTPAPath::GetLengthOfLongestPathFromApexToEdge(TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Apex)
{
	float LeftPathLength = 0.f;
	float RightPathLength = 0.f;

	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode CurrentNode = *Apex;
	while (CurrentNode.GetPrevNode() != nullptr)
	{
		LeftPathLength += FVector2D::Distance(CurrentNode.GetValue(), CurrentNode.GetPrevNode()->GetValue());
		CurrentNode = *CurrentNode.GetPrevNode();
	}

	CurrentNode = *Apex;
	while (CurrentNode.GetNextNode() != nullptr)
	{
		RightPathLength += FVector2D::Distance(CurrentNode.GetValue(), CurrentNode.GetNextNode()->GetValue());
		CurrentNode = *CurrentNode.GetNextNode();
	}

	return FMath::Max(LeftPathLength, RightPathLength);
}

float FPCGTPAPath::WalkOnRightSideOfFunnelUntilClosestPointBecomesVisible(const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* StartNode, int32 Edge)
{
	float PathLength = 0.f;
	const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode *CurrentNode = StartNode;
	FVector2D ClosestPointOnEdge = Triangulation->ClosestPointOnEdge(CurrentNode->GetValue(), Edge);
	FVector2D CurrentToOneRight = CurrentNode->GetNextNode()->GetValue() - CurrentNode->GetValue();
	FVector2D CurrentToClosestPoint = ClosestPointOnEdge - CurrentNode->GetValue();
	while (Triangulation->IsInCounterClockwiseDirectionFrom(CurrentToOneRight, CurrentToClosestPoint) &&
		(CurrentNode->GetNextNode()->GetNextNode() != nullptr))
	{
		PathLength += FVector2D::Distance(CurrentNode->GetValue(), CurrentNode->GetNextNode()->GetValue());
		CurrentNode = CurrentNode->GetNextNode();

		ClosestPointOnEdge = Triangulation->ClosestPointOnEdge(CurrentNode->GetValue(), Edge);
		CurrentToOneRight = CurrentNode->GetNextNode()->GetValue() - CurrentNode->GetValue();
		CurrentToClosestPoint = ClosestPointOnEdge - CurrentNode->GetValue();
	}

	PathLength += FVector2D::Distance(CurrentNode->GetValue(), ClosestPointOnEdge);
	return PathLength;
}

float FPCGTPAPath::WalkOnLeftSideOfFunnelUntilClosestPointBecomesVisible(const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* StartNode, int32 Edge)
{
	float PathLength = 0.f;
	const TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode *CurrentNode = StartNode;
	FVector2D ClosestPointOnEdge = Triangulation->ClosestPointOnEdge(CurrentNode->GetValue(), Edge);
	FVector2D CurrentToOneLeft = CurrentNode->GetPrevNode()->GetValue() - CurrentNode->GetValue();
	FVector2D CurrentToClosestPoint = ClosestPointOnEdge - CurrentNode->GetValue();
	while (Triangulation->IsInClockwiseDirectionFrom(CurrentToOneLeft, CurrentToClosestPoint) &&
		(CurrentNode->GetPrevNode()->GetPrevNode() != nullptr))
	{
		PathLength += FVector2D::Distance(CurrentNode->GetValue(), CurrentNode->GetPrevNode()->GetValue());
		CurrentNode = CurrentNode->GetPrevNode();

		ClosestPointOnEdge = Triangulation->ClosestPointOnEdge(CurrentNode->GetValue(), Edge);
		CurrentToOneLeft = CurrentNode->GetPrevNode()->GetValue() - CurrentNode->GetValue();
		CurrentToClosestPoint = ClosestPointOnEdge - CurrentNode->GetValue();
	}

	PathLength += FVector2D::Distance(CurrentNode->GetValue(), ClosestPointOnEdge);
	return PathLength;
}

float FPCGTPAPath::LengthOfBuiltPathInFunnel(const TDoubleLinkedList<FVector2D> &Path)
{
	float Length = 0.f;
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* CurrentNode = Path.GetHead();
	while (CurrentNode->GetNextNode() != nullptr)
	{
		Length += FVector2D::Distance(CurrentNode->GetValue(), CurrentNode->GetNextNode()->GetValue());
		CurrentNode = CurrentNode->GetNextNode();
	}
	return Length;
}

float FPCGTPAPath::MinimalDistanceBetween(int32 Edge, FVector2D TargetPoint, bool bShouldIncludePointsInTriangle, int32 Triangle)
{
	bool bPointFallsInTriangle = Triangulation->IsInTriangle(TargetPoint, Triangle);
	if (!bPointFallsInTriangle || (bPointFallsInTriangle && bShouldIncludePointsInTriangle))
	{
		return Triangulation->DistanceFromEdge(TargetPoint, Edge);
	}
	return FLT_MAX;
}

float FPCGTPAPath::MinimalDistanceBetween(FVector2D Point, FVector2D TargetPoint, bool bShouldIncludePointsInTriangle, int32 Triangle)
{
	bool bPointFallsInTriangle = Triangulation->IsInTriangle(TargetPoint, Triangle);
	if (!bPointFallsInTriangle || (bPointFallsInTriangle && bShouldIncludePointsInTriangle))
	{
		return FVector2D::Distance(Point, TargetPoint);
	}
	return FLT_MAX;
}

FPCGTPAPathFinder& FPCGTPAPathFinder::operator=(const FPCGTPAPathFinder& Other)
{
	OpenSet.Empty();	
	TDoubleLinkedList<FPCGTPAPath>::TDoubleLinkedListNode* Node = Other.OpenSet.GetHead();
	while ( Node != nullptr )
	{
		OpenSet.AddTail(Node->GetValue());
		Node = Node->GetNextNode();
	}

	HigherBounds = Other.HigherBounds;
	Triangulation = Other.Triangulation;
	return *this;
}

bool FPCGTPAPathFinder::CanTraverse(const FPCGDelaunayTriangulation& Triangulation, int32 Triangle)
{
	if (Triangulation.Flags)
	{
		return static_cast<FPCGPathfindingFlags*>(Triangulation.Flags)->CanTraverse(Triangle);
	}
	return true;
}

void FPCGTPAPathFinder::CloneDoubleLL(const TDoubleLinkedList<FVector2D> &From, TDoubleLinkedList<FVector2D> &To)
{
	To.Empty();
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Node = From.GetHead();
	while ( Node != nullptr )
	{
		To.AddTail(Node->GetValue());
		Node = Node->GetNextNode();
	}
}
/*
{
	TDoubleLinkedList<FVector2D> Path;
	bool bRet = FindPath(StartPoint, StartTriangle, Goal, Path);
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Node = Path.GetHead();
	while (Node != nullptr)
	{
		OutPath.Add(Node->GetValue());
		Node = Node->GetNextNode();
	}
	return bRet;
}*/

//bool FPCGTPAPathFinder::FindPath(const FVector2D& StartPoint, int32 StartTriangle, const FVector2D& Goal, TDoubleLinkedList<FVector2D>& OutPath)
bool FPCGTPAPathFinder::FindPath(const FVector2D& StartPoint, int32 StartTriangle, const FVector2D& Goal, TArray<FVector2D>& OutPath)
{
	OpenSet.Empty();
	OutPath.Empty();
	HigherBounds.Empty();

	TDoubleLinkedList<FVector2D> BestCandidate;
	BestCandidate.AddHead(StartPoint);
	float BestCandidateLength = FLT_MAX;

	FPCGTPAPath InitialPath = FPCGTPAPath(StartPoint, StartTriangle, Triangulation);
	AddToOpenSet(InitialPath);
	bool bDone = false;
	int32 Catch = 0;

	while ((OpenSet.Num() > 0) && !bDone)
	{
		Catch++;
		if (Catch > 10000)
		{
			UE_LOG(LogUPCGTRegionPathfinding, Warning, TEXT("TPAStar hit break limit"));
			break;
		}

		FPCGTPAPath PartialPath = OpenSet.GetHead()->GetValue();
		OpenSet.RemoveNode(OpenSet.GetHead());

		if (PartialPath.MinimalTotalCost() > BestCandidateLength)
		{
			bDone = true;
			// Copy BestCandidateLL to TArray
			TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* Node = BestCandidate.GetHead();
			while (Node != nullptr)
			{
				OutPath.Add(Node->GetValue());
				Node = Node->GetNextNode();
			}
		}
		else {
			if (!PartialPath.bReachedPathsBuilt)
			{
				if (Triangulation->IsInTriangle(Goal, PartialPath.GetCurrentTriangle()))
				{
					TDoubleLinkedList<FVector2D> NewCandidate;
					PartialPath.BuildCompletePathTo(Goal, NewCandidate);
					float NewCandidateLength = GetLength(NewCandidate);
					if (NewCandidateLength < BestCandidateLength)
					{
						CloneDoubleLL(NewCandidate, BestCandidate);
						BestCandidateLength = NewCandidateLength;
						UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("New best cadidate Path:"));
						PartialPath.DebugLog();
					}

				}
				PartialPath.bReachedPathsBuilt = true;
				PartialPath.UpdateEstimationToClosestGoalPoint(Goal);
				AddToOpenSet(PartialPath);
				UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Added a Built Path to Open Set"));
				DebugLog();
			}
			else {
				for (auto Neighbour : Triangulation->GetTriangleNeighbours(PartialPath.GetCurrentTriangle()))
				{
					if (!CanTraverse(*Triangulation, Neighbour))
					{
						continue;
					}

					if (PartialPath.GetCurrentEdge() < 0 || !(Triangulation->HasEdge(PartialPath.GetCurrentEdge(), Neighbour)))
					{
						FPCGTPAPath PathToNeighbour = PartialPath.BuildPartialPathTo(Neighbour, Goal);
						if (IsGoodCandidate(PathToNeighbour))
						{
							AddToOpenSet(PathToNeighbour);
							UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Added a Neighbour Path to Open Set"));
							DebugLog();
							UpdateHigherBoundToReachedEdge(PathToNeighbour);
						}
						else {
							UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Path with current triangle %d is not a good candidate and won't be added to open set"), PathToNeighbour.GetCurrentTriangle());
						}
					}
					else {
						UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Neighbour %d to current triangle %d is on current edge %d, and won't be added to open set"),
							Neighbour, PartialPath.GetCurrentTriangle(), PartialPath.GetCurrentEdge());
					}
				}
			}
		}
	}
	return bDone;
}

float FPCGTPAPathFinder::GetLength(const TDoubleLinkedList<FVector2D>& Path)
{
	float Length = 0.f;
	TDoubleLinkedList<FVector2D>::TDoubleLinkedListNode* CurrentNode = Path.GetHead();
	while (CurrentNode && CurrentNode->GetNextNode() != nullptr)
	{
		Length += FVector2D::Distance(CurrentNode->GetValue(), CurrentNode->GetNextNode()->GetValue());
		CurrentNode = CurrentNode->GetNextNode();
	}
	return Length;
}

bool FPCGTPAPathFinder::IsGoodCandidate(const FPCGTPAPath& Path) const
{
	const float* Length = HigherBounds.Find(Path.GetCurrentEdge());
	if (Length)
	{
		if (*Length < Path.ShortestPathToEdgeLength())
		{
			return false;
		}
	}
	return true;
}

void FPCGTPAPathFinder::UpdateHigherBoundToReachedEdge(const FPCGTPAPath& Path)
{
	float* Length = HigherBounds.Find(Path.GetCurrentEdge());
	if (Length)
	{
		if (*Length > Path.LongestPathToEdgeLength())
		{
			*Length = Path.LongestPathToEdgeLength();
		}
	}
	else {
		HigherBounds.Add(Path.GetCurrentEdge(), Path.LongestPathToEdgeLength());
	}
}

void FPCGTPAPathFinder::AddToOpenSet(const FPCGTPAPath & Path)
{
	if ((OpenSet.GetHead() == nullptr) || (OpenSet.GetHead()->GetValue().MinimalTotalCost() > Path.MinimalTotalCost()))
	{
		UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Added a Neighbour Path to beginning of open Set"));
		OpenSet.AddHead(Path);
	}
	else {
		TDoubleLinkedList<FPCGTPAPath>::TDoubleLinkedListNode* TargetNode = OpenSet.GetHead();
		while ((TargetNode->GetNextNode() != nullptr) &&
			(TargetNode->GetNextNode()->GetValue().MinimalTotalCost() < Path.MinimalTotalCost()))
		{
			TargetNode = TargetNode->GetNextNode();
		}
		if (TargetNode->GetNextNode() != nullptr)
		{
			UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Inserted a Neighbour Path to before path with current tri %d"), TargetNode->GetNextNode()->GetValue().GetCurrentTriangle());
			OpenSet.InsertNode(Path, TargetNode == nullptr ? nullptr : TargetNode->GetNextNode());
		}
		else {
			UE_LOG(LogUPCGTRegionPathfinding, VeryVerbose, TEXT("Added a Neighbour Path to end of open Set"));
			OpenSet.AddTail(Path);
		}
	}
}

void FPCGTPAPathFinder::SetTriangulation(FPCGDelaunayTriangulation* InTriangulation)
{
	Triangulation = InTriangulation;
}
