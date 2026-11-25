
function [scenario_names, scenario_obstacles, ...
    start_positions, goal_positions] = create_test_scenarios()

% ==============================================================
% CREATE TEST SCENARIOS
% APPROACH A : MULTIPLE-ENVIRONMENT VALIDATION
% ==============================================================


% STEP 1: SCENARIO NAMES

scenario_names = {
    'E1_Reference';
    'E2_Relocated_Obstacles';
    'E3_Narrow_Passage';
    'E4_Cluttered_Environment';
    'E5_Different_Start_Goal'
};


% STEP 2: INITIALIZE OBSTACLE CONFIGURATIONS

scenario_obstacles = cell(5, 1);


% --------------------------------------------------------------
% E1: REFERENCE ENVIRONMENT
% --------------------------------------------------------------

scenario_obstacles{1} = [
    3.0, 3.0, 0.40;
    6.5, 4.0, 0.50;
    5.0, 7.0, 0.35
];


% --------------------------------------------------------------
% E2: RELOCATED OBSTACLES
% --------------------------------------------------------------

scenario_obstacles{2} = [
    3.5, 6.5, 0.45;
    6.5, 3.5, 0.55;
    5.0, 5.0, 0.35
];


% --------------------------------------------------------------
% E3: NARROW PASSAGE
% --------------------------------------------------------------

scenario_obstacles{3} = [
    4.1, 5.0, 0.30;
    5.9, 5.0, 0.30
];


% --------------------------------------------------------------
% E4: CLUTTERED ENVIRONMENT
% --------------------------------------------------------------

scenario_obstacles{4} = [
    2.8, 4.0, 0.35;
    3.8, 6.0, 0.40;
    5.0, 3.0, 0.45;
    6.2, 5.0, 0.35;
    7.2, 7.0, 0.40;
    4.8, 8.0, 0.30;
    7.5, 3.0, 0.30;
    2.5, 7.5, 0.30
];


% --------------------------------------------------------------
% E5: DIFFERENT START AND GOAL POSITIONS
% --------------------------------------------------------------

scenario_obstacles{5} = [
    3.0, 6.0, 0.40;
    5.0, 5.0, 0.50;
    7.0, 3.5, 0.40
];


% STEP 3: START POSITIONS

start_positions = [
    1.5, 1.5;
    1.5, 1.5;
    5.0, 1.5;
    1.5, 1.5;
    1.5, 8.5
];


% STEP 4: GOAL POSITIONS

goal_positions = [
    8.5, 8.5;
    8.5, 8.5;
    5.0, 8.5;
    8.5, 8.5;
    8.5, 1.5
];


end
