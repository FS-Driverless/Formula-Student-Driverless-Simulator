// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public Licenseas published by the Free Software Foundation; either version 2of the License, or (at your option) any later version.This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty ofMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See theGNU General Public License for more details.You should have received a copy of the GNU General Public Licensealong with this program; if not, write to the Free SoftwareFoundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


#include "Referee.h"


// Sets default values
AReferee::AReferee()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AReferee::BeginPlay()
{
	Super::BeginPlay();
	
}

msr::airlib::CarApiBase::RefereeState AReferee::getState() {
	//todo: implement a counter and update the counter based on cone hits
	return msr::airlib::CarApiBase::RefereeState(99714);
}

// Called every frame
void AReferee::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

