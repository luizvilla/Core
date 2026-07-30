%
% Copyright (c) 2021-present LAAS-CNRS
%
%   This program is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 2 of the License, or
%   (at your option) any later version.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with this program.  If not, see <https://www.gnu.org/licenses/>.
%
% SPDX-License-Identifier: GPL-2.0-or-later
%
% @author Luiz Villa <luiz.villa@laas.fr>
%

classdef ThingSetTools < handle
    % ThingSetTools  Talk ThingSet Text Mode to a device over its
    % dedicated shell UART. MATLAB port of thingset_tools.py.
    %
    % Quick start:
    %
    %   ts = ThingSetTools();                   % port is optional: auto-
    %                                            % detected by USB VID/PID,
    %                                            % then a handshake probe
    %   tree = ts.discover();                   % walks the tree, writes
    %                                            % thingset_objects.json
    %   ts.read("Measurements/rV1Low_V")
    %   ts.write("Config", struct("wBlinkPeriod_s", 0.2))
    %
    % Auto-detection (see findPorts) first tries ports matching OwnTech's
    % USB vendor ID, falling back to every serial port on the system if
    % none match. Since a board's console and ThingSet-shell ports share
    % the same VID/PID, each candidate is actually opened and sent the
    % "select thingset" handshake - only the shell port responds.

    properties (SetAccess = private)
        Port            % connected serial port name, e.g. "/dev/ttyACM1"
        Tree            % struct built by discover(): name -> item/group metadata
    end

    properties (Access = private)
        Serial          % underlying serialport object
        Discovered = false
    end

    properties (Constant)
        % OwnTech boards' USB vendor ID. Both the console and the
        % ThingSet-shell CDC-ACM ports of the same board share this VID
        % (and usually the same PID too), so it narrows the search but
        % doesn't single out the shell port by itself - see the
        % handshake-probing loop in the constructor.
        OwnTechUsbVid = "2FE3"

        % ThingSet response status codes (thingset.io/spec, "Access
        % Functions").
        StatusCodes = containers.Map( ...
            {hex2dec('81'), hex2dec('82'), hex2dec('84'), hex2dec('85'), ...
             hex2dec('A0'), hex2dec('A1'), hex2dec('A3'), hex2dec('A4'), ...
             hex2dec('A5'), hex2dec('A8'), hex2dec('A9'), hex2dec('AD'), ...
             hex2dec('AF'), hex2dec('C0'), hex2dec('C1'), hex2dec('C4'), ...
             hex2dec('C5')}, ...
            {'Created', 'Deleted', 'Changed', 'Content', ...
             'Bad Request', 'Unauthorized', 'Forbidden (read-only value)', ...
             'Not Found', 'Method Not Allowed', 'Request Entity Incomplete', ...
             'Conflict', 'Request Entity Too Large', ...
             'Unsupported Content-Format', 'Internal Server Error', ...
             'Not Implemented', 'Gateway Timeout', 'Not a Gateway'})
    end

    methods
        function obj = ThingSetTools(port, baudRate, timeoutSeconds, vid, pid)
            % Connect to a ThingSet-over-shell device. If `port` is
            % omitted, candidates are found by USB `vid`/`pid` (see
            % findPorts), falling back to every serial port on the system
            % if none match, and each is tried in turn until one answers
            % the ThingSet handshake.
            arguments
                port (1,1) string = ""
                baudRate (1,1) double = 115200
                timeoutSeconds (1,1) double = 1.0
                vid (1,1) string = ThingSetTools.OwnTechUsbVid
                pid string = ""
            end

            obj.Tree = struct();

            if strlength(port) > 0
                obj.connect(port, baudRate, timeoutSeconds);
                return
            end

            candidates = ThingSetTools.findPorts(vid, pid);
            if isempty(candidates)
                candidates = serialportlist("available");
            end
            for i = 1:numel(candidates)
                try
                    obj.connect(candidates(i), baudRate, timeoutSeconds);
                    return
                catch
                    % try the next candidate
                end
            end
            error("ThingSetTools:notFound", ...
                "no ThingSet-over-shell device found (tried %d candidate port(s))", ...
                numel(candidates));
        end

        function close(obj)
            obj.Serial = [];
        end

        function delete(obj)
            obj.Serial = [];
        end

        % ---- ThingSet requests ---------------------------------------

        function value = get(obj, path)
            % GET a path. Returns the parsed JSON value (or a group dump).
            arguments
                obj
                path (1,1) string = ""
            end
            value = obj.parseResponse(obj.transact("?" + path, 3));
        end

        function names = fetchChildren(obj, path)
            % FETCH-with-null a group path; returns child names as a
            % string array.
            arguments
                obj
                path (1,1) string = ""
            end
            value = obj.parseResponse(obj.transact("?" + path + " null", 3));
            if iscell(value)
                % jsondecode's cell array shape isn't guaranteed to be a
                % row vector; force one so `for name = fetchChildren(...)`
                % iterates per-element rather than once over a column.
                names = reshape(string(value), 1, []);
            else
                names = string.empty;
            end
        end

        function value = read(obj, path)
            % Read a single item's value.
            value = obj.get(path);
        end

        function write(obj, path, values)
            % UPDATE `path` with `values` (a scalar struct of
            % item_name -> value). Handles the Zephyr shell's
            % unescaped-double-quote stripping automatically.
            payload = strrep(jsonencode(values), '"', '\"');
            obj.parseResponse(obj.transact("=" + path + " " + payload, 3));
        end

        % ---- discovery -------------------------------------------------

        function tree = discover(obj, jsonPath)
            % Recursively walk the device's ThingSet object tree, save it
            % to `jsonPath` (default "thingset_objects.json"; pass "" to
            % skip saving), and return it as a nested struct.
            arguments
                obj
                jsonPath (1,1) string = "thingset_objects.json"
            end
            tree = obj.discoverNode("");
            obj.Tree = tree;
            obj.Discovered = true;
            if strlength(jsonPath) > 0
                fid = fopen(jsonPath, 'w');
                fwrite(fid, jsonencode(tree, "PrettyPrint", true));
                fclose(fid);
            end
        end

        % ---- bulk read / write -----------------------------------------

        function values = readAll(obj, tree)
            % Read every readable item found by discover(), as a nested
            % struct mirroring the object tree. Calls discover() first if
            % needed.
            arguments
                obj
                tree = []
            end
            if isempty(tree)
                if ~obj.Discovered
                    tree = obj.discover();
                else
                    tree = obj.Tree;
                end
            end
            values = struct();
            names = fieldnames(tree);
            for i = 1:numel(names)
                name = names{i};
                meta = tree.(name);
                if strcmp(meta.Type, "group")
                    values.(name) = obj.readAll(meta.Children);
                elseif strcmp(meta.Type, "executable")
                    continue
                else
                    try
                        values.(name) = obj.read(meta.Path);
                    catch ME
                        values.(name) = "<error: " + string(ME.message) + ">";
                    end
                end
            end
        end

        function results = writeValues(obj, values)
            % Write `values` to the device's writable ("w"/"s") items.
            %
            % `values` may be a nested struct mirroring the discovered
            % tree (e.g. struct("Config", struct("wBlinkPeriod_s", 0.2)))
            % or a containers.Map with flat "Group/item" keys. Requires
            % discover() to have run first, so items can be checked
            % against their declared access type before anything is
            % sent. Returns a containers.Map of {path: "ok" or
            % "error: ..."}.
            if ~obj.Discovered
                obj.discover();
            end

            if isa(values, "containers.Map")
                flat = values;
            else
                flat = obj.flattenStruct(values, "");
            end

            results = containers.Map("KeyType", "char", "ValueType", "any");
            byGroup = containers.Map("KeyType", "char", "ValueType", "any");
            allPaths = keys(flat);
            for i = 1:numel(allPaths)
                path = allPaths{i};
                value = flat(path);
                meta = obj.lookup(path);
                if isempty(meta)
                    results(path) = "error: unknown ThingSet path";
                    continue
                end
                if ~any(strcmp(meta.Type, ["writable", "writable-setting"]))
                    results(path) = sprintf("error: %s is %s, not writable", path, meta.Type);
                    continue
                end
                idx = find(path == '/', 1, 'last');
                if isempty(idx)
                    parent = '';
                    leaf = path;
                else
                    parent = path(1:idx-1);
                    leaf = path(idx+1:end);
                end
                if ~isKey(byGroup, parent)
                    byGroup(parent) = struct();
                end
                grp = byGroup(parent);
                grp.(leaf) = value;
                byGroup(parent) = grp;
            end

            groupNames = keys(byGroup);
            for i = 1:numel(groupNames)
                parent = groupNames{i};
                leaves = byGroup(parent);
                leafNames = fieldnames(leaves);
                try
                    obj.write(parent, leaves);
                    for j = 1:numel(leafNames)
                        results(obj.joinPath(parent, leafNames{j})) = "ok";
                    end
                catch ME
                    for j = 1:numel(leafNames)
                        results(obj.joinPath(parent, leafNames{j})) = "error: " + string(ME.message);
                    end
                end
            end
        end
    end

    methods (Static)
        function ports = findPorts(vid, pid)
            % List serial ports matching a USB vendor ID (and optionally
            % a specific product ID). Defaults to OwnTech's VID. Reads
            % /sys/class/tty (Linux only); falls back to returning every
            % available port elsewhere. Adapted from
            % old/old4/find_devices.py.
            arguments
                vid (1,1) string = ThingSetTools.OwnTechUsbVid
                pid string = ""
            end
            allPorts = serialportlist("available");
            ports = string.empty;
            for i = 1:numel(allPorts)
                info = ThingSetTools.readUsbIds(allPorts(i));
                if isempty(info)
                    continue
                end
                if strcmpi(info.vid, vid) && (strlength(pid) == 0 || strcmpi(info.pid, pid))
                    ports(end+1) = allPorts(i); %#ok<AGROW>
                end
            end
        end
    end

    methods (Static, Access = private)
        function info = readUsbIds(portName)
            % Best-effort USB VID/PID lookup via Linux sysfs. Returns []
            % if unavailable (not Linux, or not a USB-CDC device).
            info = [];
            if ~isunix
                return
            end
            [~, devName] = fileparts(portName);
            base = "/sys/class/tty/" + devName + "/device/../";
            vidFile = base + "idVendor";
            pidFile = base + "idProduct";
            if isfile(vidFile) && isfile(pidFile)
                info = struct( ...
                    "vid", strtrim(fileread(vidFile)), ...
                    "pid", strtrim(fileread(pidFile)));
            end
        end

        function path = joinPath(parent, leaf)
            % Note: isempty("") is false for a MATLAB string scalar (it's
            % a 1x1 array whose one element happens to be empty text) -
            % only isempty('') on a char array is true. Since callers
            % pass both char (containers.Map keys) and string (path
            % building during discovery), strlength(string(...)) is the
            % only check that's correct for either input type.
            if strlength(string(parent)) == 0
                path = char(leaf);
            else
                path = [char(parent) '/' char(leaf)];
            end
        end
    end

    methods (Access = private)
        function connect(obj, port, baudRate, timeoutSeconds)
            obj.Port = port;
            obj.Serial = serialport(port, baudRate, "Timeout", timeoutSeconds);
            pause(0.3);
            flush(obj.Serial);
            obj.transact("", timeoutSeconds);
            obj.transact("select thingset", timeoutSeconds);
            obj.get("");
        end

        function tree = discoverNode(obj, path)
            tree = struct();
            names = obj.fetchChildren(path);
            for i = 1:numel(names)
                name = names(i);
                childPath = obj.joinPath(path, name);
                value = obj.get(childPath);
                fieldName = matlab.lang.makeValidName(name);
                if isstruct(value)
                    tree.(fieldName) = struct( ...
                        "Type", "group", ...
                        "Path", childPath, ...
                        "Children", obj.discoverNode(childPath));
                else
                    tree.(fieldName) = struct( ...
                        "Type", obj.classifyLeaf(name), ...
                        "Path", childPath);
                end
            end
        end

        function kind = classifyLeaf(~, name)
            chars = char(name);
            if isempty(chars)
                kind = "unknown";
                return
            end
            switch chars(1)
                case 'r'
                    kind = "read-only";
                case 'w'
                    kind = "writable";
                case 's'
                    kind = "writable-setting";
                case 'x'
                    kind = "executable";
                otherwise
                    kind = "informational";
            end
        end

        function meta = lookup(obj, path)
            parts = strsplit(path, '/');
            container = obj.Tree;
            meta = [];
            for i = 1:numel(parts)
                name = matlab.lang.makeValidName(parts{i});
                if ~isstruct(container) || ~isfield(container, name)
                    meta = [];
                    return
                end
                meta = container.(name);
                if i < numel(parts)
                    if ~isfield(meta, "Children")
                        meta = [];
                        return
                    end
                    container = meta.Children;
                end
            end
        end

        function flat = flattenStruct(obj, s, prefixPath)
            flat = containers.Map("KeyType", "char", "ValueType", "any");
            names = fieldnames(s);
            for i = 1:numel(names)
                name = names{i};
                path = obj.joinPath(prefixPath, name);
                value = s.(name);
                if isstruct(value)
                    sub = obj.flattenStruct(value, path);
                    subKeys = keys(sub);
                    for j = 1:numel(subKeys)
                        flat(subKeys{j}) = sub(subKeys{j});
                    end
                else
                    flat(path) = value;
                end
            end
        end

        function body = transact(obj, cmd, timeoutSeconds)
            flush(obj.Serial);
            write(obj.Serial, uint8([char(cmd) 13 10]), "uint8");

            ansiPat = [char(27) '\[[0-9;]*m'];
            promptPat = '[A-Za-z0-9_-]+:~\$\s*';

            t0 = tic;
            raw = uint8([]);
            plainText = "";
            found = false;
            while toc(t0) < timeoutSeconds
                n = obj.Serial.NumBytesAvailable;
                if n > 0
                    raw = [raw, read(obj.Serial, n, "uint8")]; %#ok<AGROW>
                    plainText = string(regexprep(char(raw), ansiPat, ''));
                    if ~isempty(regexp(plainText, promptPat, 'once'))
                        found = true;
                        break
                    end
                else
                    pause(0.02);
                end
            end
            if ~found
                error("ThingSetTools:timeout", "no response to '%s' on %s", cmd, obj.Port);
            end

            body = regexprep(plainText, promptPat, '');
            lines = strsplit(body, {char(13), newline});
            cmdTrim = strtrim(cmd);
            keepLines = string.empty;
            for i = 1:numel(lines)
                l = strtrim(string(lines{i}));
                if strlength(l) > 0 && l ~= cmdTrim
                    keepLines(end+1) = l; %#ok<AGROW>
                end
            end
            body = strtrim(strjoin(keepLines, newline));
        end

        function value = parseResponse(obj, body)
            % Note: the payload capture group must NOT be nested inside
            % an optional non-capturing group - MATLAB's regexp silently
            % drops it from 'tokens' output in that case (verified
            % against R2024b). \s*(.*) always matches (empty when there
            % is no payload), so no optional wrapper is needed.
            tok = regexp(char(body), ':([0-9A-Fa-f]{2})\s*(.*)$', 'tokens', 'once');
            if isempty(tok)
                error("ThingSetTools:badResponse", "unexpected response: %s", body);
            end
            code = hex2dec(tok{1});
            payloadStr = string(tok{2});
            if code >= hex2dec('A0')
                if isKey(obj.StatusCodes, code)
                    msg = obj.StatusCodes(code);
                else
                    msg = 'Unknown error';
                end
                error("ThingSetTools:status", "0x%02X %s", code, msg);
            end
            if strlength(payloadStr) == 0
                value = [];
                return
            end
            try
                value = jsondecode(char(payloadStr));
            catch
                value = payloadStr;
            end
        end
    end
end
