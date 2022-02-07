#include <iostream>

#include "Particle.h"
#include "Rigidbody.h"

/* Change this type to build with a different
 * type of simulation. */
typedef RigidbodySim sim_t;

int main()
{
	sim_t program;

	while (program.IsRunning())
	{
		program.Update();
		program.Render();
	}
	return 0;
}
