function test_connection(varargin)
    % Standalone smoke test: find the shield, open it, run the LEG1 setup
    % subsequence up to POWER_ON, read V1/V2 ten times, and park with IDLE.
    % Extra arguments are forwarded to findShieldDevicePort, e.g.
    % test_connection('ProductID','0100') to override the default PID.

    results = struct('name', {}, 'pass', {}, 'detail', {});

    try
        port = discoverPort(varargin{:});
        results(end + 1) = report('Discover port', true, sprintf('Found %s', port));
    catch ME
        results(end + 1) = report('Discover port', false, ME.message);
        printSummary(results);
        return
    end

    try
        d = ShieldDevice(port);
        results(end + 1) = report('Open device', true, sprintf('Opened %s', port));
    catch ME
        results(end + 1) = report('Open device', false, ME.message);
        printSummary(results);
        return
    end

    try
        d.sendCommand('IDLE');
        d.sendCommand('BUCK', 'LEG1', 'ON');
        d.sendCommand('LEG', 'LEG1', 'ON');
        d.sendCommand('REFERENCE', 'LEG1', 'V1', 5);
        d.sendCommand('POWER_ON');
        results(end + 1) = report('Reach POWER_ON', true, 'Setup sequence sent without error');
    catch ME
        results(end + 1) = report('Reach POWER_ON', false, ME.message);
        parked = safeIdle(d);
        results(end + 1) = report('Park with IDLE', parked, '');
        printSummary(results);
        return
    end

    try
        v1values = zeros(1, 10);
        v2values = zeros(1, 10);
        for i = 1:10
            v1values(i) = d.getMeasurement('V1');
            v2values(i) = d.getMeasurement('V2');
            pause(1);
        end
        allFinite = all(isfinite(v1values)) && all(isfinite(v2values));
        detail = sprintf('finite=%d V1=[%s] V2=[%s]', allFinite, ...
            num2str(v1values, '%.5f '), num2str(v2values, '%.5f '));
        results(end + 1) = report('Read measurements', allFinite, detail);
    catch ME
        results(end + 1) = report('Read measurements', false, ME.message);
    end

    parked = safeIdle(d);
    results(end + 1) = report('Park with IDLE', parked, '');

    printSummary(results);
end

function port = discoverPort(varargin)
    args = [{'Interactive', false}, varargin];
    port = findShieldDevicePort(args{:});
end

function ok = safeIdle(d)
    try
        d.sendCommand('IDLE');
        ok = true;
    catch
        ok = false;
    end
end

function r = report(name, pass, detail)
    if pass
        status = 'PASS';
    else
        status = 'FAIL';
    end
    fprintf('%s: %s %s\n', status, name, detail);
    r = struct('name', name, 'pass', pass, 'detail', detail);
end

function printSummary(results)
    total = numel(results);
    passed = sum([results.pass]);
    fprintf('SUMMARY: %d/%d passed\n', passed, total);
end
