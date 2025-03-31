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

	/*Grid = Cast<AGAGridActor>(UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass()));
	if (Grid)
	{
		FlowMap = FGAGridMap(Grid, 0.0f);
		UE_LOG(LogTemp, Warning, TEXT("=== Traversable Cells ==="));
		int Count = 0;
		for (int X = 0; X < Grid->XCount; ++X)
		{
			for (int Y = 0; Y < Grid->YCount; ++Y)
			{
				FCellRef Cell(X, Y);
				if (EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
				{
					UE_LOG(LogTemp, Warning, TEXT("Traversable Cell: (%d, %d)"), X, Y);
					if (++Count >= 50) break;
				}
			}
			if (Count >= 50) break;
		}
		InitializeFlowMap();
	}*/
	Grid = Cast<AGAGridActor>(UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass()));
	if (Grid)
	{
		FlowMap = FGAGridMap(Grid, 0.0f);

		// Defer initialization to allow NavMesh grid data to populate
		GetWorld()->GetTimerManager().SetTimerForNextTick(this, &URiverFlowComponent::InitializeFlowMap);
	}
}


// Called every frame
void URiverFlowComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!Grid || !FlowMap.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("FlowMap or Grid is invalid."));
		return;
	}


	SimulateFlow(DeltaTime);
	Grid->DebugGridMap = FlowMap;
	Grid->RefreshDebugTexture();

	for (int X = 0; X < 5; ++X)
	{
		float Value = 0.0f;
		FCellRef TestCell(X, 58);
		if (FlowMap.GetValue(TestCell, Value))
		{
			UE_LOG(LogTemp, Warning, TEXT("Flow at (%d, 58): %f"), X, Value);
		}
	}
}

void URiverFlowComponent::SimulateFlow(float DeltaTime)
{
	FGAGridMap NewMap(Grid, 0.0f);

	for (int Y = 58; Y <= 70; ++Y)
	{
		FCellRef Source(0, Y);
		FlowMap.SetValue(Source, 1.0f); // "spring" flow
	}

	// Limit how far water can travel in a single tick
	float StepSize = FlowStrength * DeltaTime;
	if (StepSize > 1.0f)
		StepSize = 1.0f;

	for (int X = 0; X < Grid->XCount; ++X)
	{
		for (int Y = 0; Y < Grid->YCount; ++Y)
		{
			FCellRef Cell(X, Y);
			float Water;
			if (!FlowMap.GetValue(Cell, Water)) continue;

			// Compute new position based on flow direction
			FVector2D Pos = FVector2D(X, Y) + FlowDirection * StepSize;
			FCellRef TargetCell(FMath::RoundToInt(Pos.X), FMath::RoundToInt(Pos.Y));
			// FCellRef TargetCell(FMath::FloorToInt(Pos.X + 0.5f), FMath::FloorToInt(Pos.Y + 0.5f));
			// UE_LOG(LogTemp, Warning, TEXT("Water from (%d, %d) ¡ú (%d, %d)"), X, Y, TargetCell.X, TargetCell.Y);

			if (Grid->IsCellRefInBounds(TargetCell) &&
				EnumHasAllFlags(Grid->GetCellData(TargetCell), ECellData::CellDataTraversable))
			{
				float Prev = 0.0f;
				NewMap.GetValue(TargetCell, Prev);
				NewMap.SetValue(TargetCell, Prev + Water * 0.95f);
				// UE_LOG(LogTemp, Warning, TEXT("Water source added at (%d, %d)"), Cell.X, Cell.Y);
			}
		}
	}

	FlowMap = NewMap;
}

void URiverFlowComponent::InitializeFlowMap()
{
	if (!Grid || !FlowMap.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("FlowMap not initialized properly!"));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("Initializing FlowMap"));

	//for (int X = 0; X < Grid->XCount; ++X)
	//{ 
	//	for (int Y = 0; Y < Grid->YCount; ++Y)
	//	{
	//		FCellRef Cell(X, Y);
	//		if (EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
	//		{
	//			UE_LOG(LogTemp, Warning, TEXT("Traversable Cell: (%d, %d)"), X, Y);
	//			// You can break early if you want
	//		}
	//	}
	//}

	// Example: Add water to 10 river cells in a vertical line
	for (int Y = 58; Y <= 70; ++Y)
	{
		FCellRef Cell(0, Y); // Adjust X to match river's path
		//if (Grid->IsCellRefInBounds(Cell) &&
		//	EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
		//{
		//	FlowMap.SetValue(Cell, 1.0f); // 1.0 = some initial water
		//	UE_LOG(LogTemp, Warning, TEXT("Water source added at (%d, %d)"), Cell.X, Cell.Y);
		//}
		if (!Grid->IsCellRefInBounds(Cell))
		{
			UE_LOG(LogTemp, Warning, TEXT("Cell (%d, %d) is out of bounds!"), Cell.X, Cell.Y);
			continue;
		}

		if (!EnumHasAllFlags(Grid->GetCellData(Cell), ECellData::CellDataTraversable))
		{
			UE_LOG(LogTemp, Warning, TEXT("Cell (%d, %d) is not traversable!"), Cell.X, Cell.Y);
			continue;
		}

		FlowMap.SetValue(Cell, 1.0f);
		// UE_LOG(LogTemp, Warning, TEXT("Water source added at (%d, %d)"), Cell.X, Cell.Y);
	}
}

