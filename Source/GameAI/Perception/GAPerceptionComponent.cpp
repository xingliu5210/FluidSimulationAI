#include "GAPerceptionComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GAPerceptionSystem.h"
#include "GameAI/Grid/GAGridActor.h"

UGAPerceptionComponent::UGAPerceptionComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	// Default vision parameters
	VisionParameters.VisionAngle = 90.0f;
	VisionParameters.VisionDistance = 1000.0;
}


void UGAPerceptionComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterPerceptionComponent(this);
	}
}

void UGAPerceptionComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterPerceptionComponent(this);
	}
}


APawn* UGAPerceptionComponent::GetOwnerPawn() const
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



// Returns the Target this AI is attending to right now.

UGATargetComponent* UGAPerceptionComponent::GetCurrentTarget() const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);

	if (PerceptionSystem && PerceptionSystem->TargetComponents.Num() > 0)
	{
		UGATargetComponent* TargetComponent = PerceptionSystem->TargetComponents[0];
		if (TargetComponent->IsKnown())
		{
			return PerceptionSystem->TargetComponents[0];
		}
	}

	return NULL;
}

bool UGAPerceptionComponent::HasTarget() const
{
	return GetCurrentTarget() != NULL;
}


bool UGAPerceptionComponent::GetCurrentTargetState(FTargetCache& TargetStateOut, FTargetData& TargetDataOut) const
{
	UGATargetComponent* Target = GetCurrentTarget();
	if (Target)
	{
		const FTargetData* TargetData = TargetMap.Find(Target->TargetGuid);
		if (TargetData)
		{
			TargetStateOut = Target->LastKnownState;
			TargetDataOut = *TargetData;
			return true;
		}

	}
	return false;
}


void UGAPerceptionComponent::GetAllTargetStates(bool OnlyKnown, TArray<FTargetCache>& TargetCachesOut, TArray<FTargetData>& TargetDatasOut) const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			const FTargetData* TargetData = TargetMap.Find(TargetComponent->TargetGuid);
			if (TargetData)
			{
				if (!OnlyKnown || TargetComponent->IsKnown())
				{
					TargetCachesOut.Add(TargetComponent->LastKnownState);
					TargetDatasOut.Add(*TargetData);
				}
			}
		}
	}
}


void UGAPerceptionComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateAllTargetData();
}


void UGAPerceptionComponent::UpdateAllTargetData()
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			UpdateTargetData(TargetComponent);
		}
	}
}

void UGAPerceptionComponent::UpdateTargetData(UGATargetComponent* TargetComponent)
{
	// REMEMBER: the UGAPerceptionComponent is going to be attached to the controller, not the pawn. So we call this special accessor to 
	// get the pawn that our controller is controlling
	APawn* OwnerPawn = GetOwnerPawn();
	if (OwnerPawn == NULL)
	{
		return;
	}

	FTargetData *TargetData = TargetMap.Find(TargetComponent->TargetGuid);
	if (TargetData == NULL)		// If we don't already have a target data for the given target component, add it
	{
		FTargetData NewTargetData;
		FGuid TargetGuid = TargetComponent->TargetGuid;
		TargetData = &TargetMap.Add(TargetGuid, NewTargetData);
	}

	if (TargetData)
	{
		// TODO PART 3
		// 
		// - Update TargetData->bClearLOS
		//		Use this.VisionParameters to determine whether the target is within the vision cone or not 
		//		(and ideally do so before you cast a ray towards it)
		// - Update TargetData->Awareness
		//		On ticks when the AI has a clear LOS, the Awareness should grow
		//		On ticks when the AI does not have a clear LOS, the Awareness should decay
		//
		// Awareness should be clamped to the range [0, 1]
		// You can add parameters to the UGAPerceptionComponent to control the speed at which awareness rises and falls

		// YOUR CODE HERE
		// Get world and check it's valid
		UWorld* World = GetWorld();
		if (!World) return;

		// Get Locations of AI and target
		FVector AI_Location = OwnerPawn->GetActorLocation();
		FVector Target_Location = TargetComponent->GetOwner()->GetActorLocation();
		FVector ToTarget = (Target_Location - AI_Location);
		
		// Compute squared distance (avoiding unnecessary sqrt computations)
		float DistSquared = ToTarget.SizeSquared();
		float VisionDistSquared = VisionParameters.VisionDistance * VisionParameters.VisionDistance;

		// If the target is outside the vision range, immediately lose LOS
		if (DistSquared > VisionDistSquared)
		{
			TargetData->bClearLos = false;
			TargetData->Awareness = FMath::Clamp(TargetData->Awareness - AwarenessDecayRate * World->GetDeltaSeconds(), 0.0f, 1.0f);
			return;
		}

		// Normalize direction vector and check field of view using dot product
		ToTarget.Normalize();
		float DotProduct = FVector::DotProduct(ToTarget, OwnerPawn->GetActorForwardVector());

		// Convert FOV to cos(angle) for efficient comparison
		float VisionConeCos = FMath::Cos(FMath::DegreesToRadians(VisionParameters.VisionAngle * 0.5f));
		bool bIsInVisionCone = DotProduct >= VisionConeCos;

		// Perform Line of Sight (LOS) check
		bool bHasLOS = false;
		if (bIsInVisionCone)
		{
			FHitResult HitResult;
			FCollisionQueryParams QueryParams;
			QueryParams.AddIgnoredActor(OwnerPawn); // Ignore AI itself
			QueryParams.AddIgnoredActor(TargetComponent->GetOwner()); // Ignore the target itself

			bHasLOS = !World->LineTraceSingleByChannel(
				HitResult,
				AI_Location,
				Target_Location,
				ECC_Visibility,
				QueryParams
			);
		}

		// Update LOS status
		TargetData->bClearLos = bHasLOS;

		// Awareness Calculation
		if (bHasLOS)
		{
			// Increase awareness if LOS is clear
			TargetData->Awareness = FMath::Clamp(TargetData->Awareness + AwarenessIncreaseRate * World->GetDeltaSeconds(), 0.0f, 1.0f);
		}
		else
		{
			// Decrease awareness if LOS is lost
			TargetData->Awareness = FMath::Clamp(TargetData->Awareness - AwarenessDecayRate * World->GetDeltaSeconds(), 0.0f, 1.0f);
		}

		// Debugging logs (optional)
		// UE_LOG(LogTemp, Warning, TEXT("Target Awareness: %f, LOS: %s, Distance: %f"),
			// TargetData->Awareness, TargetData->bClearLos ? TEXT("TRUE") : TEXT("FALSE"), FMath::Sqrt(DistSquared));
	}
}


const FTargetData* UGAPerceptionComponent::GetTargetData(FGuid TargetGuid) const
{
	return TargetMap.Find(TargetGuid);
}

bool UGAPerceptionComponent::TestVisibility(const FCellRef& Cell) const
{
	// Get the Target Component (assume the AI is tracking the player)
	UGATargetComponent* TargetComponent = GetCurrentTarget();
	if (!TargetComponent) return false;

	// Get the Grid Actor through the target component
	const AGAGridActor* Grid = TargetComponent->GetGridActor();
	if (!Grid) return false;

	APawn* OwnerPawn = GetOwnerPawn();
	if (!OwnerPawn) return false;

	// Get AI position and cell position
	FVector AIPosition = OwnerPawn->GetActorLocation();
	FVector CellPosition = Grid->GetCellPosition(Cell);

	// Calculate direction to the target cell
	FVector DirectionToCell = (CellPosition - AIPosition).GetSafeNormal();
	FVector ForwardVector = OwnerPawn->GetActorForwardVector();

	// Compute dot product to check if the cell is within the vision cone
	float DotProduct = FVector::DotProduct(ForwardVector, DirectionToCell);
	float VisionAngleCosine = FMath::Cos(FMath::DegreesToRadians(VisionParameters.VisionAngle * 0.5f));

	// Check if within vision cone
	if (DotProduct < VisionAngleCosine)
	{
		return false;
	}

	// Ensure the cell is within vision distance
	float DistanceToCell = FVector::Dist(AIPosition, CellPosition);
	if (DistanceToCell > VisionParameters.VisionDistance)
	{
		return false;
	}

	// Perform a line trace to ensure no obstacles block the vision
	FHitResult HitResult;
	FCollisionQueryParams QueryParams;
	QueryParams.AddIgnoredActor(OwnerPawn);

	bool bHit = GetWorld()->LineTraceSingleByChannel(
		HitResult, AIPosition, CellPosition, ECC_Visibility, QueryParams
	);

	// Return true if no obstacles block the view
	return !bHit;
}

