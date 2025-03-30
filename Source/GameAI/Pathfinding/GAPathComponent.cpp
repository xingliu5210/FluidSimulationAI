#include "GAPathComponent.h"
#include "GameFramework/NavMovementComponent.h"
#include "GameFramework/Character.h"
#include "Kismet/GameplayStatics.h"

UGAPathComponent::UGAPathComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	State = GAPS_None;
	bDestinationValid = false;
	ArrivalDistance = 100.0f;

	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


const AGAGridActor* UGAPathComponent::GetGridActor() const
{
	if (GridActor.Get())
	{
		return GridActor.Get();
	}
	else
	{
		AGAGridActor* Result = NULL;
		AActor *GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}

APawn* UGAPathComponent::GetOwnerPawn()
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}


void UGAPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (bDestinationValid)
	{
		RefreshPath();

		if (State == GAPS_Active)
		{
			FollowPath();
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("TickComponent: bDestinationValid = false! Pathfinding skipped."));
	}

	// Super important! Otherwise, unbelievably, the Tick event in Blueprint won't get called

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

EGAPathState UGAPathComponent::RefreshPath()
{
	AActor* Owner = GetOwnerPawn();
	if (Owner)
	{
		FVector StartPoint = Owner->GetActorLocation();
		const AGAGridActor* Grid = GetGridActor();
		check(bDestinationValid);

		float DistanceToDestination = FVector::Dist(StartPoint, Destination);
		// UE_LOG(LogTemp, Warning, TEXT("RefreshPath: Start = (%f, %f, %f), Destination = (%f, %f, %f)"),
		// 	StartPoint.X, StartPoint.Y, StartPoint.Z, Destination.X, Destination.Y, Destination.Z);
		// UE_LOG(LogTemp, Warning, TEXT("RefreshPath: Distance to Destination = %f"), DistanceToDestination);

		if (DistanceToDestination <= ArrivalDistance)
		{
			// Yay! We got there!
			// UE_LOG(LogTemp, Warning, TEXT("RefreshPath: Destination reached!"));
			State = GAPS_Finished;
		}
		else
		{
			TArray<FPathStep> UnsmoothedSteps;
			TMap<FCellRef, FCellRef> PrevMap;  // Store reconstructed path info
			Steps.Empty();

			// use A* or Dijkstra based on 'bUseAstar'
			bool bPathSuccess = false;
			if (bUseAStar)
			{
				// UE_LOG(LogTemp, Warning, TEXT("RefreshPath: Using A* Pathfinding."))
				State = AStar(StartPoint, UnsmoothedSteps);
				bPathSuccess = (State == GAPS_Active);
			}
			else
			{
				// UE_LOG(LogTemp, Warning, TEXT("RefreshPath: Using Dijkstra Pathfinding."));

				// Run Dijkstra to generate the distance map and path reconstruction
				FGAGridMap DistanceMap;

				bPathSuccess = Dijkstra(StartPoint, DistanceMap, PrevMap);
				if (bPathSuccess)
				{
					ReconstructPath(DestinationCell, PrevMap, UnsmoothedSteps);
				}
			}

			if (!bPathSuccess || UnsmoothedSteps.Num() == 0)
			{
				// UE_LOG(LogTemp, Error, TEXT("RefreshPath: Pathfinding FAILED!"));
				return GAPS_Invalid;
			}

			/*if (!PrevMap.Contains(DestinationCell))
			{
				UE_LOG(LogTemp, Error, TEXT("RefreshPath: ERROR - DestinationCell [%d, %d] NOT found in PrevMap!"),
					DestinationCell.X, DestinationCell.Y);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("RefreshPath: DestinationCell [%d, %d] FOUND in PrevMap! Path reconstruction should work."),
					DestinationCell.X, DestinationCell.Y);
			}*/

			// Use PrevMap to reconstruct the path
			//ReconstructPath(DestinationCell, PrevMap, UnsmoothedSteps);

			//if (UnsmoothedSteps.Num() == 0)
			//{
			//	UE_LOG(LogTemp, Error, TEXT("RefreshPath: Path reconstruction FAILED!"));
			//	return GAPS_Invalid;  // No valid path found
			//}

			//if (State == EGAPathState::GAPS_Active)
			//{
			//	// Smooth the path!
			//	State = SmoothPath(StartPoint, UnsmoothedSteps, Steps);
			//}
			// Smooth the path before following it

			// UE_LOG(LogTemp, Warning, TEXT("RefreshPath: Path reconstructed. Smoothing path..."));
			State = SmoothPath(StartPoint, UnsmoothedSteps, Steps);

			if (Steps.Num() == 0)
			{
				UE_LOG(LogTemp, Error, TEXT("RefreshPath: Smoothed path is EMPTY!"));
				State = GAPS_Invalid;
			}
			else
			{
				// UE_LOG(LogTemp, Warning, TEXT("RefreshPath: Path successfully created with %d steps."), Steps.Num());
			}
		}
	}
	else
	{
		State = GAPS_Invalid;
	}

	return State;
}

EGAPathState UGAPathComponent::AStar(const FVector& StartPoint, TArray<FPathStep>& StepsOut) const
{
	// Assignment 2 Part3: replace this with an A* search!
	// HINT 1: you made need a heap structure. A TArray can be accessed as a heap -- just add/remove elements using
	// the TArray::HeapPush() and TArray::HeapPop() methods.
	// Note that whatever you push or pop needs to implement the 'less than' operator (operator<)
	// HINT 2: UE has some useful flag testing function. For example you can test for traversability by doing this:
	// ECellData Flags = Grid->GetCellData(CellRef);
	// bool bIsCellTraversable = EnumHasAllFlags(Flags, ECellData::CellDataTraversable)
	const AGAGridActor* Grid = GetGridActor();
	if (!Grid) return GAPS_Invalid;

	// Ensure destination is valid
	if (!bDestinationValid || DestinationCell == FCellRef::Invalid)
	{
		return GAPS_Invalid;
	}

	// Clamp start point to the grid
	FCellRef StartCell = Grid->GetCellRef(StartPoint, true);

	// Early out if start is invalid
	if (StartCell == FCellRef::Invalid)
	{
		return GAPS_Invalid;
	}

	// Priority queue using TArray as a heap
	struct FHeapNode
	{
		FCellRef Cell;
		float FScore; // f(x) = g(x) + h(x)

		bool operator<(const FHeapNode& Other) const
		{
			return FScore < Other.FScore; // Min-heap
		}
	};

	TArray<FHeapNode> OpenSet; // Min-heap priority queue
	TMap<FCellRef, FCellRef> CameFrom; // Path reconstruction map, left child, right parent
	TMap<FCellRef, float> GScore; // Cost to reach each cell

	// Initialize
	GScore.Add(StartCell, 0.0f);
	OpenSet.HeapPush(FHeapNode{StartCell, static_cast<float>(FVector::Dist(StartPoint,Destination))});

	// A* Search Loop
	while (OpenSet.Num() > 0)
	{
		// Get the node with the smallest f(x)
		FHeapNode CurrentNode;
		OpenSet.HeapPop(CurrentNode);
		FCellRef CurrentCell = CurrentNode.Cell;

		// Check if we've reached the destination
		if (CurrentCell == DestinationCell)
		{
			// Path reconstruction
			TArray<FCellRef> TotalPath;

			TotalPath.Add(CurrentCell);
			while (CameFrom.Contains(CurrentCell))
			{
				CurrentCell = CameFrom[CurrentCell];
				TotalPath.Insert(CurrentCell, 0); // Prepend
			}

			// Convert TotalPath to StepsOut
			for (const FCellRef& Cell : TotalPath)
			{
				FVector PathPoint = Grid->GetCellPosition(Cell);
				FPathStep Step;
				Step.Set(PathPoint, Cell);
				StepsOut.Add(Step);
			}

			// Remove the first step (robot's current position)
			if (StepsOut.Num() > 0)
			{
				StepsOut.RemoveAt(0); // Remove 0th Step
			}

			return GAPS_Active; // Pathfinding successful
		}

		// Explore neighbors
		TArray<FCellRef> Neighbors = {
			FCellRef(CurrentCell.X + 1, CurrentCell.Y),     // Right
			FCellRef(CurrentCell.X - 1, CurrentCell.Y),     // Left
			FCellRef(CurrentCell.X, CurrentCell.Y + 1),     // Up
			FCellRef(CurrentCell.X, CurrentCell.Y - 1),     // Down
			FCellRef(CurrentCell.X + 1, CurrentCell.Y + 1), // Top-Right
			FCellRef(CurrentCell.X - 1, CurrentCell.Y + 1), // Top-Left
			FCellRef(CurrentCell.X + 1, CurrentCell.Y - 1), // Bottom-Right
			FCellRef(CurrentCell.X - 1, CurrentCell.Y - 1)  // Bottom-Left
		};

		for (const FCellRef& Neighbor : Neighbors)
		{
			// Make sure the neighbor is valid - within bounds
			if (!Grid->IsCellRefInBounds(Neighbor)) continue;

			// Check if the cell is traversable
			ECellData Flags = Grid->GetCellData(Neighbor);
			if (!EnumHasAllFlags(Flags, ECellData::CellDataTraversable)) continue;

			// Calculate tentative g-score
			float TentativeG = GScore[CurrentCell] + FVector::Dist(Grid->GetCellPosition(CurrentCell), Grid->GetCellPosition(Neighbor));

			// If this is a better path to the neighbor
			if (!GScore.Contains(Neighbor) || TentativeG < GScore[Neighbor])
			{
				CameFrom.Add(Neighbor, CurrentCell);
				GScore.Add(Neighbor, TentativeG);

				float HScore = FVector::Dist(Grid->GetCellPosition(Neighbor), Destination);
				float FScore = TentativeG + HScore;

				// Push the neighbor into the heap
				OpenSet.HeapPush(FHeapNode{ Neighbor, static_cast<float>(FScore) });
			}
		}

	}

	// if no path is found - moves directly towards the player
	StepsOut.SetNum(1);
	StepsOut[0].Set(Destination, DestinationCell);

	return GAPS_Active;
}

// Dijkstra's Algorithm to compute the shortest path distance from StartPoint
bool UGAPathComponent::Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut, TMap<FCellRef, FCellRef>& PrevMap)
{
	const AGAGridActor* Grid = GetGridActor();
	if (!Grid) return false;

	// Get starting cell reference
	FCellRef StartCell = Grid->GetCellRef(StartPoint, true);
	if (StartCell == FCellRef::Invalid) return false;

	PrevMap.Empty(); // Stores the path resconstruction

	// Priority queue (Min-Heap) for exploration
	struct FHeapNode
	{
		FCellRef Cell;
		float Cost;

		bool operator<(const FHeapNode& Other) const
		{
			return Cost < Other.Cost; // Min-heap behavior
		}
	};

	TArray<FHeapNode> OpenSet; // Min-heap
	TMap<FCellRef, float> CostMap; // Track Lowest cost per cell

	// Initialize with the starting cell
	CostMap.Add(StartCell, 0.0f);
	OpenSet.HeapPush({ StartCell, 0.0f });

	// Possible movement directions (8 neighbors for more flexible movement)
	const TArray<FCellRef> Directions = {
		{1, 0}, {-1, 0}, {0, 1}, {0, -1},  // Cardinal directions
		{1, 1}, {-1, 1}, {1, -1}, {-1, -1} // Diagonal directions
	};

	// Dijkstra's Search Loop
	while (OpenSet.Num() > 0)
	{
		// Extract cell with the lowest cost
		FHeapNode CurrentNode;
		OpenSet.HeapPop(CurrentNode);
		FCellRef CurrentCell = CurrentNode.Cell;

		// Explore neighbors
		for (const FCellRef& Offset : Directions)
		{
			FCellRef Neighbor = { CurrentCell.X + Offset.X, CurrentCell.Y + Offset.Y };

			// Ensure neighbor is within grid bounds and traversable
			if (!Grid->IsCellRefInBounds(Neighbor)) continue;
			ECellData Flags = Grid->GetCellData(Neighbor);
			if (!EnumHasAllFlags(Flags, ECellData::CellDataTraversable)) continue;

			// Compute cost (assuming uniform traversal cost)
			float NewCost = CostMap[CurrentCell] + FVector::Dist(Grid->GetCellPosition(CurrentCell), Grid->GetCellPosition(Neighbor));

			// If the new cost is lower than the stored cost, update it
			if (!CostMap.Contains(Neighbor) || NewCost < CostMap[Neighbor])
			{
				CostMap.Add(Neighbor, NewCost);
				PrevMap.Add(Neighbor, CurrentCell); // Track Path reconstruction
				OpenSet.HeapPush({ Neighbor, NewCost });
			}
		}
	}

	// Store results in the distance map
	for (const TPair<FCellRef, float>& Entry : CostMap)
	{
		DistanceMapOut.SetValue(Entry.Key, Entry.Value);
	}

	return true;

}

void UGAPathComponent::ReconstructPath(const FCellRef& EndCell, const TMap<FCellRef, FCellRef>& PrevMap, TArray<FPathStep>& PathOut) const
{
	const AGAGridActor* Grid = GetGridActor();
	if (!Grid) return;

	FCellRef Current = EndCell;

	// Backtrack from EndCell to StartCell using PrevMap
	while (PrevMap.Contains(Current))
	{
		FVector PathPoint = Grid->GetCellPosition(Current);

		FPathStep Step;
		Step.Set(PathPoint, Current);
		PathOut.Insert(Step, 0); // Prepend the path
		Current = PrevMap[Current]; // Move to previous cell
	}
}

EGAPathState UGAPathComponent::SmoothPath(const FVector& StartPoint, const TArray<FPathStep>& UnsmoothedSteps, TArray<FPathStep>& SmoothedStepsOut) const
{
	// Assignment 2 Part 4: smooth the path
	// High level description from the lecture:
	// * Trace to each subsequent step until you collide
	// * Back up one step (to the last clear one)
	// * Add that cell to your smoothed step
	// * Start again from there
	const AGAGridActor* Grid = GetGridActor();
	if (!Grid) return GAPS_Invalid;

	// Handle empty path edge case
	if (UnsmoothedSteps.Num() == 0) return GAPS_Invalid;

	SmoothedStepsOut.Empty();

	// Apex point of the funnel
	FVector Apex = UnsmoothedSteps[0].Point;

	// Initialize left and right edges of the funnel
	FVector LeftEdge = Apex;
	FVector RightEdge = Apex;

	// Add the starting point to the smoothed path
	SmoothedStepsOut.Add(UnsmoothedSteps[0]);

	for (int32 i = 1; i < UnsmoothedSteps.Num(); ++i)
	{
		FVector CurrentPoint = UnsmoothedSteps[i].Point;

		// Update the funnel
		if (FVector::CrossProduct(Apex - CurrentPoint, RightEdge -CurrentPoint).Z < 0)
		{
			RightEdge = CurrentPoint;

			// If the funnel collapses (right edge crosses left edge)
			if (FVector::CrossProduct(Apex - CurrentPoint, RightEdge - CurrentPoint).Z > 0)
			{
				// Add the apex to the smoothed path and reset the funnel
				FPathStep TempStep;
				TempStep.Set(Apex, UnsmoothedSteps[i - 1].CellRef);
				SmoothedStepsOut.Add(TempStep);

				Apex = LeftEdge;
				LeftEdge = Apex;
				RightEdge = CurrentPoint;
			}
		}

		if (FVector::CrossProduct(Apex - CurrentPoint, RightEdge - CurrentPoint).Z > 0)
		{
			LeftEdge = CurrentPoint;

			// If the funnel collapses (left edge crosses right edge)
			if (FVector::CrossProduct(Apex - CurrentPoint, RightEdge - CurrentPoint).Z < 0)
			{
				// Add the apex to the smoothed path and reset the funnel
				FPathStep TempStep;
				TempStep.Set(Apex, UnsmoothedSteps[i - 1].CellRef);
				SmoothedStepsOut.Add(TempStep);

				Apex = RightEdge;
				RightEdge = Apex;
				LeftEdge = CurrentPoint;
			}
		}
	}

	// Add the final destination to the smoothed path
	SmoothedStepsOut.Add(UnsmoothedSteps.Last());

	return GAPS_Active;
}

bool UGAPathComponent::LineTrace(const FCellRef& Start, const FCellRef& End, const AGAGridActor* Grid) const
{
	// Performs a line trace between two grid cells to check if there is a clear path (no non-traversable cells).
	// Make sure Grid is valid
	if (!Grid) return false;

	// Get the FVectors of the two points
	FVector StartPoint = Grid->GetCellPosition(Start);
	FVector EndPoint = Grid->GetCellPosition(End);

	// Log the StartPoint and EndPoint for debugging
	// UE_LOG(LogTemp, Warning, TEXT("LineTrace StartPoint: X=%f, Y=%f, Z=%f"), StartPoint.X, StartPoint.Y, StartPoint.Z);
	// UE_LOG(LogTemp, Warning, TEXT("LineTrace EndPoint: X=%f, Y=%f, Z=%f"), EndPoint.X, EndPoint.Y, EndPoint.Z);


	// Calculate the number of steps for interpolation
	float RequiredSteps = FVector::Dist(StartPoint, EndPoint) / Grid->CellScale; 

	// Safety check for zero steps
	if (RequiredSteps <= 0) return true;

	// Line trace through interpolated points
	for (int i = 0; i <= RequiredSteps; ++i)
	{
		float Alpha = static_cast<float>(i) / RequiredSteps; // Fractional interpolation value
		FVector Point = FMath::Lerp(StartPoint, EndPoint, Alpha);
		FCellRef Cell = Grid->GetCellRef(Point, true);

		if (!Grid->IsCellRefInBounds(Cell) || !EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
		{
			return false; // Line of sight is blocked
		}

	}

	return true; // Line of sight is clear
}


void UGAPathComponent::FollowPath()
{
	AActor* Owner = GetOwnerPawn();
	if (Owner)
	{
		FVector StartPoint = Owner->GetActorLocation();

		check(State == GAPS_Active);
		//check(Steps.Num() > 0);

		//// Always follow the first step, assuming that we are refreshing the whole path every tick
		//FVector V = Steps[0].Point - StartPoint;
		//V.Normalize();

		//UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
		//if (MovementComponent)
		//{
		//	MovementComponent->RequestPathMove(V);
		//}
		if (Steps.Num() > 0)
		{
			// Always follow the first step, assuming that we are refreshing the whole path every tick
			FVector V = Steps[0].Point - StartPoint;
			V.Normalize();

			UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
			if (MovementComponent)
			{
				MovementComponent->RequestPathMove(V);
			}
		}
	}
	else 
	{
		return;
	}
}


EGAPathState UGAPathComponent::SetDestination(const FVector &DestinationPoint, bool bUseAStarPath)
{
	// UE_LOG(LogTemp, Warning, TEXT("SetDestination: Called! Setting destination to (%f, %f, %f)"),
	// DestinationPoint.X, DestinationPoint.Y, DestinationPoint.Z);

	// UE_LOG(LogTemp, Warning, TEXT("SetDestination: DestinationCell = [%d, %d], IsValid = %s"),
	// 	DestinationCell.X, DestinationCell.Y, DestinationCell.IsValid() ? TEXT("true") : TEXT("false"));

	bUseAStar = bUseAStarPath;

	Destination = DestinationPoint;

	State = GAPS_Invalid;
	bDestinationValid = true;

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FCellRef CellRef = Grid->GetCellRef(Destination);
		if (CellRef.IsValid())
		{
			DestinationCell = CellRef;
			bDestinationValid = true;

			RefreshPath();
		}
	}

	return State;
}