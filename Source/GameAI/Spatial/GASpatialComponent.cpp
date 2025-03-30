#include "GASpatialComponent.h"
#include "GameAI/Pathfinding/GAPathComponent.h"
#include "GameAI/Grid/GAGridMap.h"
#include "Kismet/GameplayStatics.h"
#include "Math/MathFwd.h"
#include "GASpatialFunction.h"
#include "ProceduralMeshComponent.h"
#include "GameAI/Perception/GAPerceptionComponent.h"



UGASpatialComponent::UGASpatialComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	SampleDimensions = 8000.0f;		// should cover the bulk of the test map
}


const AGAGridActor* UGASpatialComponent::GetGridActor() const
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

const UGAPathComponent* UGASpatialComponent::GetPathComponent() const
{
	UGAPathComponent* Result = PathComponent.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* Owner = GetOwner();
		if (Owner)
		{
			// Note, the UGAPathComponent and the UGASpatialComponent are both on the controller
			Result = Owner->GetComponentByClass<UGAPathComponent>();
			if (Result)
			{
				PathComponent = Result;
			}
		}
		return Result;
	}
}

APawn* UGASpatialComponent::GetOwnerPawn() const
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


bool UGASpatialComponent::ChoosePosition(bool PathfindToPosition, bool Debug)
{
	bool Result = false;
	const APawn* OwnerPawn = GetOwnerPawn();
	const AGAGridActor* Grid = GetGridActor();
	PathComponent = GetPathComponent();

	UGAPerceptionComponent* PerceptionComp = GetOwner()->FindComponentByClass<UGAPerceptionComponent>();
	if (!PerceptionComp || !PathComponent.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("GASpatialComponent: Missing Perception or PathComponent."));
		return false;
	}

	FTargetCache TargetCache;
	FTargetData TargetData;
	FVector TargetLocation;

	// If AI has perceived the target, use last known location
	if (PerceptionComp->HasTarget() && PerceptionComp->GetCurrentTargetState(TargetCache, TargetData))
	{
		TargetLocation = TargetCache.Position;
	}
	else
	{
		return false; // No valid target to pursue
	}

	if (!PathComponent.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("PathComponent is invalid after assignment."));
	}

	if (SpatialFunctionReference.Get() == NULL)
	{
		// UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent has no SpatialFunctionReference assigned."));
		return false;
	}

	if (GridActor == NULL)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent::ChoosePosition can't find a GridActor."));
		return false;
	}

	if (!PathComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent::ChoosePosition can't find a valid PathComponent."));
		return false;
	}

	// Don't worry too much about the Unreal-ism below. Technically our SpatialFunctionReference is not ACTUALLY
	// a spatial function instance, rather it's a class, which happens to have a lot of data in it.
	// Happily, Unreal creates, under the hood, a default object for every class, that lets you access that data
	// as if it were a normal instance
	const UGASpatialFunction* SpatialFunction = SpatialFunctionReference->GetDefaultObject<UGASpatialFunction>();

	// The below is to create a GridMap (which you will fill in) based on a bounding box centered around the OwnerPawn

	FBox2D Box(EForceInit::ForceInit);
	FIntRect CellRect;
	FVector2D PawnLocation(OwnerPawn->GetActorLocation());
	Box += PawnLocation;
	Box = Box.ExpandBy(SampleDimensions / 2.0f);
	if (GridActor->GridSpaceBoundsToRect2D(Box, CellRect))
	{
		// Super annoying, by the way, that FIntRect is not blueprint accessible, because it forces us instead
		// to make a separate bp-accessible FStruct that represents _exactly the same thing_.
		FGridBox GridBox(CellRect);

		// This is the grid map I'm going to fill with values
		FGAGridMap GridMap(Grid, GridBox, 0.0f);

		// Fill in this distance map using Dijkstra!
		FGAGridMap DistanceMap(Grid, GridBox, FLT_MAX);


		// ~~~ STEPS TO FILL IN FOR ASSIGNMENT 3 ~~~

		// Step 1: Run Dijkstra's to determine which cells we should even be evaluating (the GATHER phase)
		// (You should add a Dijkstra() function to the UGAPathComponent())
		// I would recommend adding a method to the path component which looks something like
		// bool UGAPathComponent::Dijkstra(const FVector &StartPoint, FGAGridMap &DistanceMapOut) const;
		TMap<FCellRef, FCellRef> PrevMap;
		PathComponent->Dijkstra(OwnerPawn->GetActorLocation(), DistanceMap, PrevMap);

		// Step 2: For each layer in the spatial function, evaluate and accumulate the layer in GridMap
		// Note, only evaluate accessible cells found in step 1
		for (const FFunctionLayer& Layer : SpatialFunction->Layers)
		{
			// figure out how to evaluate each layer type, and accumulate the value in the GridMap
			EvaluateLayer(Layer, GridMap, DistanceMap, TargetLocation);
		}

		// Step 3: pick the best cell in GridMap
		FCellRef BestCell = FCellRef::Invalid;
		float BestValue = -FLT_MAX;
		
		for (int32 Y = GridMap.GridBounds.MinY; Y < GridMap.GridBounds.MaxY; Y++)
		{
			for (int32 X = GridMap.GridBounds.MinX; X < GridMap.GridBounds.MaxX; X++)
			{
				FCellRef CellRef(X, Y);
				float CellValue = 0.0f;

				// Check if the cell is traversable and has a valid value
				if (EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable) &&
					GridMap.GetValue(CellRef, CellValue))
				{
					// Select the highest scoring cell
					if (CellValue > BestValue)
					{
						BestValue = CellValue;
						BestCell = CellRef;
					}
				}
			}
		}

		// Let's pretend for now we succeeded.
		// Result = true;

		if (PathfindToPosition)
		{
			// Step 4: Go there!
			// This will involve reconstructing the path and then getting it into the UGAPathComponent
			// Depending on what your cached Dijkstra data looks like, the path reconstruction might be implemented here
			// or in the UGAPathComponent
			if (BestCell.IsValid())
			{
				FVector BestPosition = Grid->GetCellPosition(BestCell);
				// UE_LOG(LogTemp, Warning, TEXT("Best Position Selected: X=%f, Y=%f, Z=%f"), BestPosition.X, BestPosition.Y, BestPosition.Z);

				PathComponent->SetDestination(BestPosition, false);
				// Create path reconstruction
				TArray<FPathStep> UnsmoothedSteps;
				PathComponent->ReconstructPath(BestCell, PrevMap, UnsmoothedSteps);

				// **Smooth the Path**
				PathComponent->SmoothPath(OwnerPawn->GetActorLocation(), UnsmoothedSteps, PathComponent->Steps);

				Result = true;
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent::ChoosePosition - No valid cell found!"));
			}
		}

		
		if (Debug)
		{
			// Note: this outputs (basically) the results of the position selection
			// However, you can get creative with the debugging here. For example, maybe you want
			// to be able to examine the values of a specific layer in the spatial function
			// You could create a separate debug map above (where you're doing the evaluations) and
			// cache it off for debug rendering. Ideally you'd be able to control what layer you wanted to 
			// see from blueprint

			GridActor->DebugGridMap = GridMap;
			// GridActor->DebugGridMap = DistanceMap;
			GridActor->RefreshDebugTexture();
			GridActor->DebugMeshComponent->SetVisibility(true);		//cheeky!
		}
	}

	return Result;
}


void UGASpatialComponent::EvaluateLayer(const FFunctionLayer& Layer, FGAGridMap &GridMap, const FGAGridMap& DistanceMap, FVector TargetLocation) const
{
	AActor* OwnerPawn = GetOwnerPawn();
	const AGAGridActor* Grid = GetGridActor();
	// APawn* PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
	UWorld* World = GetWorld();

	if (!OwnerPawn || !Grid || !World) return;

	for (int32 Y = GridMap.GridBounds.MinY; Y < GridMap.GridBounds.MaxY; Y++)
	{
		for (int32 X = GridMap.GridBounds.MinX; X < GridMap.GridBounds.MaxX; X++)
		{
			FCellRef CellRef(X, Y);
			float Value = 0.0f;

			if (EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
			{
				// evaluate me!

				// First step is determine input value. Remember there are three possible inputs to handle:
				// 	SI_None				UMETA(DisplayName = "None"),
				//	SI_TargetRange		UMETA(DisplayName = "Target Range"),
				//	SI_PathDistance		UMETA(DisplayName = "PathDistance"),
				//	SI_LOS				UMETA(DisplayName = "Line Of Sight")

				// float Value = 0.0f;

				switch (Layer.Input)
				{
					case SI_None:
					{
						Value = 0.0f;
						break;
					}

					case SI_TargetRange:
					{
						// Compute Euclidean distance from AI cell to Player
						FVector CellPosition = Grid->GetCellPosition(CellRef);
						// Value = FVector::Dist(CellPosition, PlayerPawn->GetActorLocation());
						Value = FVector::Dist(CellPosition, TargetLocation);
						break;
					}

					case SI_PathDistance:
					{
						// Get path distance from precomputed Dijkstra map
						/*float PathDistance = FLT_MAX;
						if (DistanceMap.GetValue(CellRef, PathDistance))
						{
							Value = PathDistance;
						}*/
						DistanceMap.GetValue(CellRef, Value);
						break;
					}

					case SI_LOS:
					{
						// **Perform Line Trace for LOS Check**
						FHitResult HitResult;
						FCollisionQueryParams Params;
						FVector Start = Grid->GetCellPosition(CellRef);	// Start at the grid cell position
						Start.Z = TargetLocation.Z;
						// FVector End = PlayerPawn->GetActorLocation();	// End at the player¡¯s position

						// Ensure that the trace happens at the same height (avoid Z discrepancies)
						// Start.Z = End.Z;

						// Ignore AI and Player during the trace
						// Params.AddIgnoredActor(PlayerPawn);
						Params.AddIgnoredActor(OwnerPawn);

						// Perform Unreal's visibility line trace
						bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, TargetLocation, ECollisionChannel::ECC_Visibility, Params);

						// If nothing was hit, there is a clear line of sight
						Value = bHitSomething ? 0.0f : 1.0f;  // 1.0 = clear, 0.0 = blocked
						break;
					}
					case SI_Blur:
					{
						float TotalValue = 0.0f;
						int Count = 0;

						for (int dY = -1; dY <= 1; dY++)
						{
							for (int dX = -1; dX <= 1; dX++)
							{
								FCellRef NeighborRef(CellRef.X + dX, CellRef.Y + dY);
								float NeighborValue = 0.0f;

								if (GridMap.GetValue(NeighborRef, NeighborValue)) // Get neighbor's value
								{
									TotalValue += NeighborValue;
									Count++;
								}
							}
						}

						Value = (Count > 0) ? (TotalValue / Count) : 0.0f;
						break;
					}

					default:
						break;
				}


				// Next, run it through the response curve using something like this
				// float Value = 4.5f;
				// float ModifiedValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(Value, 0.0f);
				float ModifiedValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(Value, 0.0f);

				// Then add it's influence to the grid map, combining with the current value using one of the two operators
				//	SO_None				UMETA(DisplayName = "None"),
				//	SO_Add				UMETA(DisplayName = "Add"),			// add this layer to the accumulated buffer
				//	SO_Multiply			UMETA(DisplayName = "Multiply")		// multiply this layer into the accumulated buffer

				//GridMap.SetValue(CellRef, CombinedValue);

				float ExistingValue = 0.0f;
				GridMap.GetValue(CellRef, ExistingValue);

				switch (Layer.Op)
				{
					case SO_Add:
						GridMap.SetValue(CellRef, ExistingValue + ModifiedValue);
						break;
					case SO_Multiply:
						GridMap.SetValue(CellRef, ExistingValue * ModifiedValue);
						break;
					default:
						break;
				}

				// HERE ARE SOME ADDITIONAL HINTS

				// Here's how to get the player's pawn
				// APawn *PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);

				// Here's how to cast a ray

				// UWorld* World = GetWorld();
				// FHitResult HitResult;
				// FCollisionQueryParams Params;
				// FVector Start = Grid->GetCellPosition(CellRef);		// need a ray start
				// FVector End = PlayerPawn->GetActorLocation();		// need a ray end
				// Start.Z = End.Z;		// Hack: we don't have Z information in the grid actor -- take the player's z value and raycast against that
				// Add any actors that should be ignored by the raycast by calling
				// Params.AddIgnoredActor(PlayerPawn);			// Probably want to ignore the player pawn
				// Params.AddIgnoredActor(OwnerPawn);			// Probably want to ignore the AI themself
				// bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
				// If bHitSomething is false, then we have a clear LOS
			}
		}
	}
}