// Fill out your copyright notice in the Description page of Project Settings.


#include "RiverFlowComponent.h"
#include "Kismet/GameplayStatics.h"

// Sets default values for this component's properties
URiverFlowComponent::URiverFlowComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void URiverFlowComponent::BeginPlay()
{
	Super::BeginPlay();

	Grid = Cast<AGAGridActor>(UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass()));
	if (Grid)
	{
		FlowMap = FGAGridMap(Grid, 0.0f);
		InitializeFlowMap();
	}
}


// Called every frame
void URiverFlowComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!Grid || !FlowMap.IsValid()) return;

	SimulateFlow(DeltaTime);
	Grid->DebugGridMap = FlowMap;
	Grid->RefreshDebugTexture();
}

void URiverFlowComponent::SimulateFlow(float DeltaTime)
{
	FGAGridMap NewMap(Grid, 0.0f);

	for (int X = 0; X < Grid->XCount; ++X)
	{
		for (int Y = 0; Y < Grid->YCount; ++Y)
		{
			FCellRef Cell(X, Y);
			float Water;
			if (!FlowMap.GetValue(Cell, Water)) continue;

			// Compute new position based on flow direction
			FVector2D Pos = FVector2D(X, Y) + FlowDirection * FlowStrength * DeltaTime;
			FCellRef TargetCell(FMath::RoundToInt(Pos.X), FMath::RoundToInt(Pos.Y));

			if (Grid->IsCellRefInBounds(TargetCell) &&
				EnumHasAllFlags(Grid->GetCellData(TargetCell), ECellData::CellDataTraversable))
			{
				float Prev = 0.0f;
				NewMap.GetValue(TargetCell, Prev);
				NewMap.SetValue(TargetCell, Prev + Water * 0.95f);
			}
		}
	}

	FlowMap = NewMap;
}

void URiverFlowComponent::InitializeFlowMap()
{
	if (!Grid || !FlowMap.IsValid()) return;

	// Example: Add water to 10 river cells in a vertical line
	for (int Y = 40; Y <= 50; ++Y)
	{
		FCellRef Cell(10, Y); // Adjust X to match river's path
		if (Grid->IsCellRefInBounds(Cell) &&
			EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
		{
			FlowMap.SetValue(Cell, 1.0f); // 1.0 = some initial water
		}
	}
}

