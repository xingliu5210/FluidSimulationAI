#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPathComponent.generated.h"



USTRUCT(BlueprintType)
struct FPathStep
{
	GENERATED_USTRUCT_BODY()

	FPathStep() : Point(FVector::ZeroVector), CellRef(FCellRef::Invalid) {}

	void Set(const FVector& PointIn, const FCellRef& CellRefIn)
	{
		Point = PointIn;
		CellRef = CellRefIn;
	}

	UPROPERTY(BlueprintReadWrite)
	FVector Point;

	UPROPERTY(BlueprintReadWrite)
	FCellRef CellRef;
};

// Note the UMeta -- DisplayName is just a nice way to show the name in the editor
UENUM(BlueprintType)
enum EGAPathState
{
	GAPS_None			UMETA(DisplayName = "None"),
	GAPS_Active			UMETA(DisplayName = "Active"),
	GAPS_Finished		UMETA(DisplayName = "Finished"),
	GAPS_Invalid		UMETA(DisplayName = "Invalid"),
};


// Our custom path following component, which will rely on the data
// contained in the GridActor
// Note the meta-specific "BlueprintSpawnableComponnet". This will allow us
// to attach this component to any actor type in Blueprint. Otherwise it would
// only be attachable in code.

UCLASS(BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class UGAPathComponent : public UActorComponent
{
	GENERATED_UCLASS_BODY()

	// Note just a cached pointer
	UPROPERTY()
	mutable TSoftObjectPtr<AGAGridActor> GridActor;

	UFUNCTION(BlueprintCallable)
	const AGAGridActor *GetGridActor() const;

	// It is super easy to forget: this component will usually be attached to the CONTROLLER, not the pawn it's controlling
	// A lot of times we want access to the pawn (e.g. when sending signals to its movement component).
	UFUNCTION(BlueprintCallable, BlueprintPure)
	APawn *GetOwnerPawn();

	UFUNCTION(BlueprintCallable)
	bool Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut, TMap<FCellRef, FCellRef>& PrevMap);

	UFUNCTION(BlueprintCallable)
	bool FlowDijkstra(const FVector& StartPoint, FGAGridMap& FlowMapOut);

	UFUNCTION(BlueprintCallable)
	void ReconstructPath(const FCellRef& EndCell, const TMap<FCellRef, FCellRef>& PrevMap, TArray<FPathStep>& PathOut) const;

	UFUNCTION(BlueprintCallable)
	// Performs a line trace between two grid cells to check if there is a clear path (no non-traversable cells).
	bool LineTrace(const FCellRef& Start, const FCellRef& End, const AGAGridActor* Grid) const;


	// State Update ------------------------

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	EGAPathState RefreshPath();

	EGAPathState AStar(const FVector& StartPoint, TArray<FPathStep>& StepsOut) const;

	EGAPathState SmoothPath(const FVector& StartPoint, const TArray<FPathStep>& UnsmoothedSteps, TArray<FPathStep>& SmoothedStepsOut) const;

	void FollowPath();

	// Parameters ------------------------

	// When I'm within this distance of my destination, my path is considered finished.
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float ArrivalDistance;

	// Destination ------------------------

	UFUNCTION(BlueprintCallable)
	EGAPathState SetDestination(const FVector &DestinationPoint, bool bUseAStarPath);

	UPROPERTY(BlueprintReadOnly)
	bool bDestinationValid;

	UPROPERTY(BlueprintReadOnly)
	bool bUseAStar;

	UPROPERTY(BlueprintReadOnly)
	FVector Destination;

	UPROPERTY(BlueprintReadOnly)
	FCellRef DestinationCell;

	UPROPERTY(BlueprintReadOnly)
	FGAGridMap FlowDistanceMap;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	AActor* RiverStart;

	// State ------------------------

	UPROPERTY(BlueprintReadOnly)
	TEnumAsByte<EGAPathState> State;

	UPROPERTY(BlueprintReadWrite)
	TArray<FPathStep> Steps;

};