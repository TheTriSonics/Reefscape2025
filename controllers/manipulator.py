
""""
This is a state machine that will control the manipulator.
"""
import enum
import ntcore
from magicbot import StateMachine, state, tunable, feedback, will_reset_to

from components.photoeye import PhotoEyeComponent
from components.arm import ArmComponent
from components.elevator import ElevatorComponent
from components.wrist import WristComponent
from components.drivetrain import DrivetrainComponent

from controllers.intake import IntakeControl

from utilities.game import ManipLocations, ManipLocation, GamePieces, is_auton, is_red
from utilities import pb
from utilities.waypoints import Waypoints


class Manipulator(StateMachine):
    wrist: WristComponent
    arm: ArmComponent
    elevator: ElevatorComponent
    photoeye: PhotoEyeComponent
    intake_control: IntakeControl
    drivetrain: DrivetrainComponent

    operator_advance = will_reset_to(False)
    reef_limit = tunable(1.5)
    
    # Create some default targets for the robot. The operator can change these
    # over in robot.py with their controller.
    game_piece_mode: GamePieces = GamePieces.CORAL
    coral_scoring_target = ManipLocations.CORAL_REEF_4
    algae_scoring_target = ManipLocations.BARGE_6
    algae_intake_target = ManipLocations.ALGAE_REEF_1
    initial_target = ManipLocations.CORAL_REEF_4

    # This is where the system will try and drive itself to at any given time
    _target_location: ManipLocation = ManipLocations.HOME

    def __init__(self):
        self.game_mode = (
            ntcore.NetworkTableInstance.getDefault()
            .getStringTopic("/components/manipulator/game_mode")
            .publish()
        )
        self.coral_scoring_level = (
            ntcore.NetworkTableInstance.getDefault()
            .getIntegerTopic("/components/manipulator/coral_level")
            .publish()
        )
        self.algae_intake_level = (
            ntcore.NetworkTableInstance.getDefault()
            .getIntegerTopic("/components/manipulator/algae_level")
            .publish()
        )
        self.algae_scoring_level = (
            ntcore.NetworkTableInstance.getDefault()
            .getStringTopic("/components/manipulator/algae_target")
            .publish()
        )
        self.game_mode.set('coral')
        self.coral_scoring_level.set(4)
        self.algae_intake_level.set(1)
        self.algae_scoring_level.set('barge')

    # This gets called by the MagicBot framework when the system is enabled
    def on_enable(self):
        print('Manipulator enabled')
        # Wherever we're at when enabled we'll just assume we should stay at
        self._target_location = ManipLocation(
            self.wrist.get_position(),
            self.arm.get_position(),
            self.elevator.get_position()
        )

    # This begins a sections of methods meant to be exposed to the operator
    # to control the manipulator. Buttons and such can be mapped to these to
    # configure how it operates.

    def request_advance(self):
        print('advance requested by operator')
        self.operator_advance = True

    def coral_mode(self):
        self.game_piece_mode = GamePieces.CORAL
        self.game_mode.set('coral')
        self.go_home()

    def algae_mode(self):
        self.game_piece_mode = GamePieces.ALGAE
        self.game_mode.set('algae')
        self.go_home()

    def go_home(self):
        # Set the necessary targets for each component
        if self.get_reef_distance() > self.reef_limit:
            self.request_location(ManipLocations.HOME)
            self.next_state(self.idling)
        self.intake_control.go_idle()
        self.engage()

    def go_algae_score(self):
        self.next_state_now(self.algae_score)
        self.engage()

    def go_algae_prepare_intake(self):
        # Protect against re-triggering it
        if self.current_state != self.algae_prepare_intake.name:
            self.next_state(self.algae_prepare_intake)
        self.engage()
    
    def go_algae_prepare_score(self):
        if self.current_state != self.algae_prepare_score.name:
            self.next_state(self.algae_prepare_score)
        self.engage()

    def go_coral_prepare_score(self):
        if self.current_state != self.coral_prepare_score.name:
            self.next_state(self.coral_prepare_score)
        self.engage()

    def go_coral_score(self):
        self.next_state_now(self.coral_score)
        self.engage()
    
    def set_coral_level1(self):
        self.coral_scoring_level.set(1)
        self.coral_scoring_target = ManipLocations.CORAL_REEF_1

    def set_coral_level2(self):
        self.coral_scoring_level.set(2)
        self.coral_scoring_target = ManipLocations.CORAL_REEF_2

    def set_coral_level3(self):
        self.coral_scoring_level.set(3)
        self.coral_scoring_target = ManipLocations.CORAL_REEF_3

    def set_coral_level4(self):
        self.coral_scoring_level.set(4)
        self.coral_scoring_target = ManipLocations.CORAL_REEF_4
    
    def set_algae_level1(self):
        self.algae_intake_level.set(1)
        self.algae_intake_target = ManipLocations.ALGAE_REEF_1

    def set_algae_level2(self):
        self.algae_intake_level.set(2)
        self.algae_intake_target = ManipLocations.ALGAE_REEF_2
    
    def set_algae_processor(self):
        self.algae_scoring_level.set('processor')
        self.algae_scoring_target = ManipLocations.PROCESSOR_5

    def set_algae_barge(self):
        self.algae_scoring_level.set('barge')
        self.algae_scoring_target = ManipLocations.BARGE_6

    def get_reef_distance(self) -> float:
        pose = self.drivetrain.get_pose()
        return Waypoints.get_distance_to_reef_center(pose, is_red()) 

    def check_limits(self):
        # Elevator----------------------------------------
        # if self.arm.target_pos < -65 and (self.elevator.get_position() > 2 or not self.wrist.at_goal()):
        #     self.arm.lower_limit = -65
        # else:
        #     self.arm.lower_limit = -90
        # This limits should not change!
        # ------------------------------------------------
        pass  # Do nothing for now

    # That's the end of the operator interface portion

    # Now some methods that the states within the system will use as helpers
    
    def go_arm_safe(self):
        if self.arm.get_position() >0:
            self.arm.target_pos = self.arm.upper_limit
        else:
            self.arm.target_pos = ManipLocations.HOME.arm_pos

    def request_location(self, location: ManipLocation):
        from copy import copy
        self._target_location = copy(location)
        self.wrist.target_pos = self._target_location.wrist_pos
        self.arm.target_pos = self._target_location.arm_pos
        self.elevator.target_pos = self._target_location.elevator_pos

    # Check to see if the system is at the target position, or close enough
    # with some deadbanding. We'll leave the 'close enough' up to the
    # individual components to decide since they know their own units and
    # are responsible for getting to the target.
    @feedback
    def at_position(self) -> bool:
        epos = self.elevator.get_position()
        apos = self.arm.get_position()
        wpos = self.wrist.get_position()
        curr_location = ManipLocation(epos, apos, wpos)
        manip_at_position = self._target_location == curr_location
        # print(f'Compared {self._target_location} to {curr_location} and got {manip_at_position}')
        return manip_at_position
    
    # That's the end of the helper methods and from here down we have the
    # various states of the state machine itself.


    # We'll start off idle; do nothing  until the operator requests something
    @state(must_finish=True, first=True)
    def idling(self, initial_call):
        if self.photoeye.front_photoeye and self.game_piece_mode == GamePieces.CORAL:
            self.next_state(self.coral_in_system)
        elif self.photoeye.front_photoeye and self.game_piece_mode == GamePieces.ALGAE:
            self.next_state(self.algae_in_system)
        else:
            if self.operator_advance:
                if self.game_piece_mode == GamePieces.CORAL:
                    self.next_state(self.coral_intake)
                elif self.game_piece_mode == GamePieces.ALGAE:
                    self.next_state(self.algae_intake)
    
    @state(must_finish=True)
    def coral_intake(self, state_tm, initial_call):
        if initial_call:
            self.request_location(ManipLocations.INTAKE_CORAL)
            self.intake_control.go_coral_intake()
        if self.photoeye.front_photoeye:
            self.next_state(self.coral_in_system)

    @state(must_finish=True)
    def coral_in_system(self, state_tm, initial_call):
        if initial_call and self.get_reef_distance() > self.reef_limit:
            arm_pos = self.coral_scoring_target.arm_pos
            if arm_pos > 0:
                self.request_location(ManipLocations.DRIVE_UP)
            else:
                self.request_location(ManipLocations.HOME)
        if self.operator_advance:
            self.next_state(self.coral_prepare_score)

    @state(must_finish=True)
    def coral_prepare_score(self, initial_call, state_tm):
        move_allowed = False
        if initial_call:
            self.initial_target = self.coral_scoring_target
            self.request_location(self.coral_scoring_target)

        # The operator could change the target value while we're in this state
        # so check for that!  First check if we're allowed to move between positions
        if self.initial_target != self.coral_scoring_target:
            if self.coral_scoring_target in [
                ManipLocations.CORAL_REEF_1,
                ManipLocations.CORAL_REEF_2,
                ManipLocations.CORAL_REEF_3
                ] and self.initial_target in [
                    ManipLocations.CORAL_REEF_1,
                    ManipLocations.CORAL_REEF_2,
                    ManipLocations.CORAL_REEF_3
                    ]:
                move_allowed = True
                initial_target = self.coral_scoring_target
            elif self.coral_scoring_target in [
                ManipLocations.CORAL_REEF_3,
                ManipLocations.CORAL_REEF_3_UP,
                ManipLocations.CORAL_REEF_4,
                ] and self.initial_target in [
                    ManipLocations.CORAL_REEF_3_UP,
                    ManipLocations.CORAL_REEF_4,
                    ]:
                if self.coral_scoring_target == ManipLocations.CORAL_REEF_3:
                    self.coral_scoring_target = ManipLocations.CORAL_REEF_3_UP
                move_allowed = True
                initial_target = self.coral_scoring_target

        if move_allowed:
            self.go_arm_safe()
            if self.arm.at_goal():
                self.elevator.target_pos = self.coral_scoring_target.elevator_pos
                if self.elevator.at_goal():
                    self.request_location(self.coral_scoring_target)

        # Here we can check if we're at the position or if we've been
        # waiting too long and we should just move on, like maybe we just can't
        # quite get to the right position, but we've got to try something
        at_pos = self.at_position() # or state_tm > 3.0 JRD - I don't think we need this
        if (self.operator_advance or is_auton()) and (at_pos):
            self.next_state(self.coral_score)

    # JJB: I'm not thrilled with the names of these states, coral_score and
    # coral_scored are too similar, but they make sense.
    @state(must_finish=True)
    def coral_score(self, state_tm, initial_call):
        # Let's score a coral!
        if initial_call:
            reverse = False
            if self.coral_scoring_target in [
                ManipLocations.CORAL_REEF_2,
                ManipLocations.CORAL_REEF_3,
            ]:
                reverse = True
            pb('Reverse score', reverse)
            self.intake_control.go_coral_score(reverse=reverse)

        # Wait until the intake controller thinks it has scored the coral
        scored = self.intake_control.current_state == self.intake_control.idling.name
        if scored:
            self.next_state(self.coral_scored)

    # NOTE: This step might not really be needed, we could return back
    # to the home position when the scoring state knows the coral has ejected
    @state(must_finish=True)
    def coral_scored(self, initial_call, state_tm):
        if initial_call:
            self.go_arm_safe()
        if self.arm.at_goal():
            self.elevator.target_pos = ManipLocations.HOME.elevator_pos
        if self.get_reef_distance() > self.reef_limit and (self.operator_advance or is_auton()):
            self.go_home()

# this is the algae stuff
    @state(must_finish=True)
    def algae_intake(self, state_tm, initial_call):
        if initial_call:
            self.intake_control.go_algae_intake()
            self.request_location(self.algae_intake_target)
        if self.photoeye.algae_held or self.operator_advance:
            self.next_state(self.algae_in_system)
        
    @state(must_finish=True) 
    def algae_in_system(self, state_tm, initial_call):
        # Wait here until the operator wants to get into scoring position
        if initial_call:
            self.intake_control.go_algae_hold()
        if self.get_reef_distance() > self.reef_limit:
            arm_pos = self.algae_scoring_target.arm_pos
            if arm_pos > 0:
                self.request_location(ManipLocations.DRIVE_UP)
            
            """ Do we want to drive here with algae?
            else:
                self.request_location(ManipLocations.HOME)
            """
        if self.operator_advance and self.get_reef_distance() > self.reef_limit:
            self.next_state(self.algae_prepare_score)
    
    @state(must_finish=True)
    def algae_prepare_score(self, initial_call, state_tm):
        if initial_call:
            self.intake_control.go_algae_hold()
            self.request_location(self.algae_scoring_target)

        # The operator could change the target value while we're in this state
        # so check for that!
        if self._target_location != self.algae_scoring_target:
            self.request_location(self.algae_scoring_target)

        # Here we can check if we're at the position or if we've been
        # waiting too long and we should just move on, like maybe we just can't
        # quite get to the right position, but we've got to try something
        if self.operator_advance and (self.at_position() or state_tm > 2.0):
            self.next_state(self.algae_score)

    @state(must_finish=True)
    def algae_prepare_intake(self, initial_call, state_tm):
        if initial_call:
            self.request_location(self.algae_intake_target)

        # The operator could change the target value while we're in this state
        # so check for that!
        if self._target_location != self.algae_intake_target:
            self.request_location(self.algae_intake_target)

        # Here we can check if we're at the position or if we've been
        # waiting too long and we should just move on, like maybe we just can't
        # quite get to the right position, but we've got to try something
        if (self.operator_advance or is_auton()) and (self.at_position() or state_tm > 2.0):
            self.next_state(self.algae_intake)

    # JJB: I'm not thrilled with the names of these states, coral_score and
    # coral_scored are too similar, but they make sense.
    @state(must_finish=True)
    def algae_score(self, state_tm, initial_call):
        # Let's score a algae!
        if initial_call:
            self.intake_control.go_algae_score()

        if self.operator_advance:
            self.next_state(self.idling)

    # NOTE: This step might not really be needed, we could return back
    # to the home position when the scoring state knows the coral has ejected
    @state(must_finish=True)
    def algae_scored(self, initial_call, state_tm):
        if initial_call:
            # What do we do after we score a coral?
            # Send ourself back into 'home' mode
            # Do we need to turn off the intake here?
            self.request_location(ManipLocations.HOME)
        # Now ask the system to start moving. When it arrives at the HOME
        # location it'll again wait for the operator to advance

        # Wait for the lifts to get back to home before we move on
        # to the idling state where we wait on user input to begin intake
        if self.at_position():
            self.next_state(self.idling)