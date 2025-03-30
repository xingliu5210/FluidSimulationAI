// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameAI/Grid/GAGridActor.h"
#include "RiverFlowComponent.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class GAMEAI_API URiverFlowComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	URiverFlowComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float FlowStrength = 0.2f; // Flow speed per second

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector2D FlowDirection = FVector2D(1.0f, 0.0f); // Rightward

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	AGAGridActor* Grid;
	FGAGridMap FlowMap;

	void InitializeFlowMap();
	void SimulateFlow(float DeltaTime);

		
};
