# f110_central_ctrler

There is one timer to periodically publish drive message cmd at a fixed frequency (100 Hz).

NMPC controller is used to generate trajectory and its corresponding control strategy. It updates the drive message cmd once get a new trajectory. 
