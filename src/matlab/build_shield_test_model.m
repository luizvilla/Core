function build_shield_test_model(outputPath)
    % Programmatically builds shield_test_model.slx: a minimal Simulink
    % model wiring ShieldSendBlock and ShieldGetBlock together, reproducing
    % comm_script.m's triangular reference ramp as an actual block diagram.
    % Regeneratable from this script rather than hand-drawn, so the model's
    % construction is code-reviewable like everything else in src/matlab/.
    %
    % build_shield_test_model() saves shield_test_model.slx next to this
    % script. build_shield_test_model(outputPath) saves it at outputPath
    % instead (used by no-hardware/real-hardware test runs that want a
    % throwaway copy).

    if nargin < 1
        outputPath = fullfile(fileparts(mfilename('fullpath')), 'shield_test_model.slx');
    end

    modelName = 'shield_test_model';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    load_system(modelName);

    % Reproduces comm_script.m's triangular ramp: reference = reference +
    % 0.5 each step, wrapping to 5 once it would reach 15. That is exactly
    % the repeating 20-value sequence below (5.5, 6.0, ..., 14.5, then 5.0
    % on the step where the Python/MATLAB loop would have hit 15).
    refSequence = [5.5:0.5:14.5, 5.0];

    refSource = add_block('simulink/Sources/Repeating Sequence Stair', ...
        [modelName '/Reference'], 'Position', [30 30 130 60]);
    set_param(refSource, 'OutValues', mat2str(refSequence), 'tsamp', '1');

    sendBlock = add_block('simulink/User-Defined Functions/MATLAB System', ...
        [modelName '/ShieldSendBlock'], 'Position', [200 20 340 80]);
    set_param(sendBlock, 'System', 'ShieldSendBlock');

    getBlock = add_block('simulink/User-Defined Functions/MATLAB System', ...
        [modelName '/ShieldGetBlock'], 'Position', [200 140 340 200]);
    set_param(getBlock, 'System', 'ShieldGetBlock');

    scope = add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
        'Position', [420 140 460 200]);
    set_param(scope, 'NumInputPorts', '2');

    toWsV1 = add_block('simulink/Sinks/To Workspace', [modelName '/V1_ToWorkspace'], ...
        'Position', [420 240 520 270]);
    set_param(toWsV1, 'VariableName', 'V1_log', 'SaveFormat', 'Array');

    toWsV2 = add_block('simulink/Sinks/To Workspace', [modelName '/V2_ToWorkspace'], ...
        'Position', [420 300 520 330]);
    set_param(toWsV2, 'VariableName', 'V2_log', 'SaveFormat', 'Array');

    toWsRef = add_block('simulink/Sinks/To Workspace', [modelName '/Ref_ToWorkspace'], ...
        'Position', [200 260 300 290]);
    set_param(toWsRef, 'VariableName', 'Ref_log', 'SaveFormat', 'Array');

    % Reference feeds both ShieldSendBlock inputs and its own logger.
    add_line(modelName, 'Reference/1', 'ShieldSendBlock/1', 'autorouting', 'on');
    add_line(modelName, 'Reference/1', 'ShieldSendBlock/2', 'autorouting', 'on');
    add_line(modelName, 'Reference/1', 'Ref_ToWorkspace/1', 'autorouting', 'on');

    % V1/V2 feed the Scope and their own loggers.
    add_line(modelName, 'ShieldGetBlock/1', 'Scope/1', 'autorouting', 'on');
    add_line(modelName, 'ShieldGetBlock/1', 'V1_ToWorkspace/1', 'autorouting', 'on');
    add_line(modelName, 'ShieldGetBlock/2', 'Scope/2', 'autorouting', 'on');
    add_line(modelName, 'ShieldGetBlock/2', 'V2_ToWorkspace/1', 'autorouting', 'on');

    set_param(modelName, ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'FixedStepDiscrete', ...
        'FixedStep', '1', ...
        'StopTime', '5');
    % Makes To Workspace logs retrievable from sim()'s returned
    % Simulink.SimulationOutput (via get(simOut, 'V1_log') etc.) instead of
    % only through the base workspace, avoiding polluting it.
    set_param(modelName, 'ReturnWorkspaceOutputs', 'on');

    save_system(modelName, outputPath);
    close_system(modelName, 0);
end
