function comm_script(varargin)
    % MATLAB port of comm_script.py: drives both legs of a Twist board in
    % buck mode with a ramping triangular voltage reference and plots the
    % measured V1/V2 live. Board is always returned to IDLE on exit --
    % normal completion, error, figure close, or Ctrl+C -- via onCleanup.
    %
    % Name-value options (all optional):
    %   FrameLimit  - frames per plot window before it resets (default 200)
    %   MaxCycles   - number of FrameLimit-frame windows to run before
    %                 stopping even if the figure is still open; default
    %                 Inf matches comm_script.py's "run until closed"
    %                 behavior. Set to a finite value for scripted/batch runs.
    %   EnablePlot  - false skips all figure/plotting calls, for headless
    %                 verification runs (default true)
    %   VendorID, ProductID, Interactive - forwarded to findShieldDevicePort

    p = inputParser;
    addParameter(p, 'FrameLimit', 200);
    addParameter(p, 'MaxCycles', Inf);
    addParameter(p, 'EnablePlot', true);
    addParameter(p, 'VendorID', '2fe3');
    addParameter(p, 'ProductID', '0101');
    addParameter(p, 'Interactive', true);
    parse(p, varargin{:});

    frameLimit = p.Results.FrameLimit;
    maxCycles = p.Results.MaxCycles;
    enablePlot = p.Results.EnablePlot;

    port = findShieldDevicePort('VendorID', p.Results.VendorID, ...
        'ProductID', p.Results.ProductID, 'Interactive', p.Results.Interactive);
    d = ShieldDevice(port);
    cleanupObj = onCleanup(@() safeIdle(d));

    refBase = 5;
    refStep = 0.5;
    refMax = 15;
    reference = refBase;

    fig = [];
    ax = [];
    line1 = [];
    line2 = [];
    if enablePlot
        fig = figure;
        ax = axes(fig);
        line1 = animatedline(ax, 'Color', [0 0.4470 0.7410], 'DisplayName', 'V1');
        line2 = animatedline(ax, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'V2');
        xlim(ax, [0 frameLimit]);
        ylim(ax, [0 14]);
        xlabel(ax, 'Time');
        ylabel(ax, 'Value');
        title(ax, 'Real-time Plot');
        legend(ax);
        grid(ax, 'on');
    end

    d.sendCommand('IDLE');
    d.sendCommand('BUCK', 'LEG1', 'ON');
    d.sendCommand('BUCK', 'LEG2', 'ON');
    d.sendCommand('LEG', 'LEG1', 'ON');
    d.sendCommand('LEG', 'LEG2', 'ON');
    d.sendCommand('REFERENCE', 'LEG1', 'V1', 5);
    d.sendCommand('POWER_ON');

    totalFrame = 0;
    cycle = 0;
    while cycle < maxCycles && (~enablePlot || isvalid(fig))
        for frame = 0:(frameLimit - 1)
            if enablePlot && ~isvalid(fig)
                break
            end

            reference = reference + refStep;
            if reference == refMax
                reference = refBase;
            end

            d.sendCommand('REFERENCE', 'LEG1', 'V1', reference);
            d.sendCommand('REFERENCE', 'LEG2', 'V2', reference);
            pause(10e-3);

            v1 = d.getMeasurement('V1');
            v2 = d.getMeasurement('V2');

            if enablePlot
                addpoints(line1, totalFrame, v1);
                addpoints(line2, totalFrame, v2);
                drawnow limitrate;
            end

            totalFrame = totalFrame + 1;
        end

        if enablePlot && isvalid(fig)
            clearpoints(line1);
            clearpoints(line2);
            xlim(ax, [totalFrame, totalFrame + frameLimit]);
        end

        cycle = cycle + 1;
    end
end

function safeIdle(d)
    try
        d.sendCommand('IDLE');
    catch
    end
end
