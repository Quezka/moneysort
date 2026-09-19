"""Money Sorter: vision-guided 3-axis stepper arm on a Raspberry Pi.

Layered package (dependencies point inward):
  domain/     pure logic - planning, kinematics, config (no lgpio, no HTTP)
  hardware/   lgpio drivers - Stepper (STEP/DIR execution)
  app/        orchestration - Arm, ArmController (e-stop, serialize)
  interface/  delivery - HTTP server, dashboard page, CLI
"""
