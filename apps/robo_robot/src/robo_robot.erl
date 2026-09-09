-module(robo_robot).

-export([start/2]).


%%--------------------------------------------------------------------
%% Config
%%--------------------------------------------------------------------

server_host() ->
    application:get_env(robo_robot, server_host, "localhost").

server_port() ->
    application:get_env(robo_robot, server_port, 5555).

step_delay_ms() ->
    application:get_env(robo_robot, step_delay_ms, 500).


%%--------------------------------------------------------------------
%% Start
%%--------------------------------------------------------------------

start(RobotId, Current) ->

    io:format("Robot ~p starting at ~p~n", [RobotId, Current]),

    {ok, Socket} =
        gen_tcp:connect(
            server_host(),
            server_port(),
            [binary, {packet, 4}, {active, false}]
        ),

    io:format("Robot ~p connected to server~n", [RobotId]),

    gen_tcp:send(Socket, term_to_binary({hello, RobotId, Current})),

    case gen_tcp:recv(Socket, 0) of

        {ok, Data} ->

            Response = binary_to_term(Data),

            io:format("Robot ~p received: ~p~n", [RobotId, Response]),

            wait_for_command(Socket, RobotId);

        Error ->

            io:format(
                "Robot ~p disconnected while waiting for initial "
                "response: ~p~n",
                [RobotId, Error]
            ),

            gen_tcp:close(Socket)
    end.


%%--------------------------------------------------------------------
%% Wait for commands from server
%%--------------------------------------------------------------------

wait_for_command(Socket, RobotId) ->

    case gen_tcp:recv(Socket, 0) of

        {ok, Data} ->

            Command = binary_to_term(Data),

            io:format(
                "Robot ~p received command: ~p~n",
                [RobotId, Command]
            ),

            handle_command(Socket, RobotId, Command),

            wait_for_command(Socket, RobotId);

        {error, closed} ->
            io:format("Robot ~p disconnected~n", [RobotId]),
            gen_tcp:close(Socket);

        {error, Reason} ->
            io:format("Robot ~p TCP error: ~p~n", [RobotId, Reason]),
            gen_tcp:close(Socket)
    end.


%%--------------------------------------------------------------------
%% Commands from server
%%--------------------------------------------------------------------

handle_command(Socket, RobotId, {go, Goal, Positions}) ->
    io:format("Robot ~p assigned goal ~p~n", [RobotId, Goal]),
    move_window(Socket, RobotId, Positions, undefined);

handle_command(Socket, RobotId, {wait, Reason}) ->
    io:format("Robot ~p waiting: ~p~n", [RobotId, Reason]),
    gen_tcp:send(Socket, term_to_binary({wait_ack, RobotId}));

handle_command(Socket, RobotId, Command) ->
    io:format("Robot ~p received unknown command: ~p~n", [RobotId, Command]),
    gen_tcp:send(Socket, term_to_binary({error, {unknown_command, Command}})).


%%--------------------------------------------------------------------
%% Walk the current window.
%%
%% `PendingGo' carries a `{go, Goal, Positions}' that arrived from the
%% server EARLY -- before we finished walking the current window --
%% because the server now prefetches the next window before our last
%% step in this one is even executed. We keep carrying it forward
%% until the current window is done, then start on it immediately
%% instead of waiting for another message.
%%--------------------------------------------------------------------

move_window(_Socket, _RobotId, [], undefined) ->
    ok;

move_window(Socket, RobotId, [], {go, Goal, Positions}) ->
    io:format(
        "Robot ~p already had its next window queued: ~p~n",
        [RobotId, Positions]
    ),
    handle_command(Socket, RobotId, {go, Goal, Positions});

move_window(Socket, RobotId, [Position | Rest], PendingGo) ->

    %% Simulated travel time for one grid cell.
    timer:sleep(step_delay_ms()),

    io:format("Robot ~p moving to ~p~n", [RobotId, Position]),

    gen_tcp:send(Socket, term_to_binary({moved, RobotId, Position})),

    case wait_for_move_result(Socket, RobotId, PendingGo) of

        {ack, NewPendingGo} ->
            move_window(Socket, RobotId, Rest, NewPendingGo);

        {done, Position} ->
            io:format(
                "Robot ~p reached its goal at ~p~n",
                [RobotId, Position]
            ),
            ok;

        {denied, Reason} ->
            io:format("Robot ~p move denied: ~p~n", [RobotId, Reason]),
            ok;

        {conn_error, Reason} ->
            io:format("Robot ~p connection issue: ~p~n", [RobotId, Reason]),
            ok
    end.


%%--------------------------------------------------------------------
%% Keep receiving until we get the actual outcome of OUR move
%% (move_ack / goal_reached / move_denied). Along the way we might see
%% an unsolicited `{go, ...}' -- the server's early prefetch -- which
%% we stash as PendingGo and keep waiting past.
%%--------------------------------------------------------------------

wait_for_move_result(Socket, RobotId, PendingGo) ->

    case gen_tcp:recv(Socket, 0) of

        {ok, Data} ->

            case binary_to_term(Data) of

                {go, Goal, Positions} ->
                    io:format(
                        "Robot ~p received next window early: ~p~n",
                        [RobotId, Positions]
                    ),
                    wait_for_move_result(
                        Socket, RobotId, {go, Goal, Positions}
                    );

                {move_ack, _Position} ->
                    {ack, PendingGo};

                {goal_reached, Position} ->
                    {done, Position};

                {move_denied, Reason} ->
                    {denied, Reason};

                Other ->
                    io:format(
                        "Robot ~p received unexpected response: ~p~n",
                        [RobotId, Other]
                    ),
                    wait_for_move_result(Socket, RobotId, PendingGo)
            end;

        {error, closed} ->
            {conn_error, closed};

        {error, Reason} ->
            {conn_error, Reason}
    end.