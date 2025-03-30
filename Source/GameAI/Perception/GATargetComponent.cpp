#include "GATargetComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPerceptionSystem.h"
#include "ProceduralMeshComponent.h"



UGATargetComponent::UGATargetComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	SetTickGroup(ETickingGroup::TG_PostUpdateWork);

	// Generate a new guid
	TargetGuid = FGuid::NewGuid();
}


AGAGridActor* UGATargetComponent::GetGridActor() const
{
	AGAGridActor* Result = GridActor.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
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


void UGATargetComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterTargetComponent(this);
	}

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		OccupancyMap = FGAGridMap(Grid, 0.0f);
	}
}

void UGATargetComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterTargetComponent(this);
	}
}



void UGATargetComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	bool isImmediate = false;

	// update my perception state FSM
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGAPerceptionComponent>> &PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
		for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
		{
			const FTargetData* TargetData = PerceptionComponent->GetTargetData(TargetGuid);
			if (TargetData && (TargetData->Awareness >= 1.0f))
			{
				isImmediate = true;
				break;
			}
		}
	}

	if (isImmediate)
	{
		AActor* Owner = GetOwner();
		LastKnownState.State = GATS_Immediate;

		// REFRESH MY STATE
		LastKnownState.Set(Owner->GetActorLocation(), Owner->GetVelocity());

		// Tell the omap to clear out and put all the probability in the observed location
		OccupancyMapSetPosition(LastKnownState.Position);
	}
	else if (IsKnown())
	{
		LastKnownState.State = GATS_Hidden;
	}

	if (LastKnownState.State == GATS_Hidden)
	{
		OccupancyMapUpdate();
	}

	// As long as I'm known, whether I'm immediate or not, diffuse the probability in the omap

	if (IsKnown())
	{
		OccupancyMapDiffuse();
	}

	if (bDebugOccupancyMap)
	{
		AGAGridActor* Grid = GetGridActor();
		Grid->DebugGridMap = OccupancyMap;
		GridActor->RefreshDebugTexture();
		GridActor->DebugMeshComponent->SetVisibility(true);
	}
}


void UGATargetComponent::OccupancyMapSetPosition(const FVector& Position)
{
	// TODO PART 4

	// We've been observed to be in a given position
	// Clear out all probability in the omap, and set the appropriate cell to P = 1.0

	// Get the Grid Actor
	AGAGridActor* Grid = GetGridActor();
	if (!Grid) return;

	// Clear the entire occupancy map
	OccupancyMap.ResetData(0.0f);

	// Convert position to the closest grid cell
	FCellRef TargetCell = Grid->GetCellRef(Position, true);
	if (TargetCell.IsValid())
	{
		// Set probability at the observed position to 100%
		OccupancyMap.SetValue(TargetCell, 1.0f);
	}

}


void UGATargetComponent::OccupancyMapUpdate()
{
	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FGAGridMap VisibilityMap(Grid, 0.0f);

		// TODO PART 4

		// STEP 1: Build a visibility map, based on the perception components of the AIs in the world
		// The visibility map is a simple map where each cell is either 0 (not currently visible to ANY perceiver) or 1 (currently visible to one or more perceivers).
		// 

		UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
		if (PerceptionSystem)
		{
			TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
			for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
			{
				// Find visible cells for this perceiver.
				// Reminder: Use the PerceptionComponent.VisionParameters when determining whether a cell is visible or not (in addition to a line trace).
				// Suggestion: you might find it useful to add a UGAPerceptionComponent::TestVisibility method to the perception component.
				for (int X = 0; X < Grid->XCount; X++)
				{
					for (int Y = 0; Y < Grid->YCount; Y++)
					{
						FCellRef Cell(X, Y);
						if (PerceptionComponent->TestVisibility(Cell))
						{
							VisibilityMap.SetValue(Cell, 1.0f);
						}
					}
				}
			}
		}


		// STEP 2: Clear out the probability in the visible cells
		float Pculled = 0.0f;
		for (int i = 0; i < OccupancyMap.Data.Num(); i++)
		{
			if (VisibilityMap.Data[i] > 0)  // If cell is visible
			{
				Pculled += OccupancyMap.Data[i];  // Sum up probability of visible cells
				OccupancyMap.Data[i] = 0.0f;      // Clear visible cells
			}
		}

		// STEP 3: Renormalize the OMap, so that it's still a valid probability distribution
		/*float Sum = 0.0f;
		for (float Value : OccupancyMap.Data)
		{
			Sum += Value;
		}

		if (Sum > 0)
		{
			for (float& Value : OccupancyMap.Data)
			{
				Value /= Sum;
			}
		}*/
		if (Pculled < 1.0f) // Avoid division by zero
		{
			float ScaleFactor = 1.0f / (1.0f - Pculled);
			for (float& Value : OccupancyMap.Data)
			{
				Value *= ScaleFactor;
			}
		}

		// STEP 4: Extract the highest-likelihood cell on the omap and refresh the LastKnownState.
		float MaxValue = -UE_MAX_FLT;
		FCellRef BestCell;
		for (int X = 0; X < Grid->XCount; X++)
		{
			for (int Y = 0; Y < Grid->YCount; Y++)
			{
				FCellRef Cell(X, Y);
				float Value;
				if (OccupancyMap.GetValue(Cell, Value) && Value > MaxValue)
				{
					MaxValue = Value;
					BestCell = Cell;
				}
			}
		}

		LastKnownState.Position = Grid->GetCellPosition(BestCell);

	}

}


void UGATargetComponent::OccupancyMapDiffuse()
{
	// TODO PART 4
	// Diffuse the probability in the OMAP

	const AGAGridActor* Grid = GetGridActor();
	if (!Grid) return;

	// Create a temporary buffer for new probability values
	FGAGridMap DiffusedMap(Grid, 0.0f);

	// Diffusion factor 
	const float DiffusionRate = 0.2f;  // ¦Á in the pseudocode

	// Iterate through all cells in the occupancy map
	for (int X = 0; X < Grid->XCount; X++)
	{
		for (int Y = 0; Y < Grid->YCount; Y++)
		{
			FCellRef Cell(X, Y);
			float CurrentValue;

			// Skip non-traversable cells
			if (!EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
			{
				continue;
			}

			// Get current probability value
			if (!OccupancyMap.GetValue(Cell, CurrentValue))
			{
				continue; // Skip if value retrieval failed
			}

			float RemainingProbability = CurrentValue; // pLEFT

			// Define the 8 neighboring cells
			TArray<FCellRef> Neighbors = {
				FCellRef(X + 1, Y),     // Right
				FCellRef(X - 1, Y),     // Left
				FCellRef(X, Y + 1),     // Up
				FCellRef(X, Y - 1),     // Down
				FCellRef(X + 1, Y + 1), // Top-Right (Diagonal)
				FCellRef(X - 1, Y + 1), // Top-Left (Diagonal)
				FCellRef(X + 1, Y - 1), // Bottom-Right (Diagonal)
				FCellRef(X - 1, Y - 1)  // Bottom-Left (Diagonal)
			};

			// Distribute probability to neighbors
			for (const FCellRef& Neighbor : Neighbors)
			{
				// Skip out-of-bounds or non-traversable cells early
				if (!Grid->IsCellRefInBounds(Neighbor) ||
					!EnumHasAllFlags(Grid->GetCellData(Neighbor), ECellData::CellDataTraversable))
				{
					continue;
				}

				float DiffuseAmount = DiffusionRate * CurrentValue;

				// Reduce diffusion for diagonal neighbors (1 / sqrt(2))
				if (Neighbor.X != X && Neighbor.Y != Y)
				{
					DiffuseAmount /= FMath::Sqrt(2.0f);
				}

				float NeighborValue = 0.0f;
				if (DiffusedMap.GetValue(Neighbor, NeighborValue))
				{
					DiffusedMap.SetValue(Neighbor, NeighborValue + DiffuseAmount);
				}

				RemainingProbability -= DiffuseAmount;
			}

			// Store the remaining probability in the original cell
			float CellValue = 0.0f;
			if (DiffusedMap.GetValue(Cell, CellValue))
			{
				DiffusedMap.SetValue(Cell, CellValue + RemainingProbability);
			}
		}
	}

	// Copy the buffer back to the occupancy map
	OccupancyMap = DiffusedMap;
}
