-module(robo_robot).

-export([start/2]).

-define(SERVER, "localhost").
-define(PORT,   5555).


start(RobotId, Current) ->
    io:format(
      "Robot ~p starting at ~p~n",
      [RobotId, Current]),

    {ok, Socket} = gen_tcp:connect(
                     ?SERVER,
                     ?PORT,
                     [binary,
                      {packet, 4},
                      {active, false}]),

    io:format(
      "Robot ~p connected to server~n",
      [RobotId]),

    gen_tcp:send(
      Socket,
      term_to_binary({hello, RobotId, Current})),

    TcpResponse = gen_tcp:recv(Socket, 0),
    case TcpResponse of
        {ok, Data} ->
            Response = binary_to_term(Data),
            io:format(
              "Robot ~p received: ~p~n",
              [RobotId, Response]),
            wait_for_command(Socket, RobotId);
        _ ->
            io:format(
              "Robot ~p disconnected while waiting for initial response~n",
              [TcpResponse]),
            gen_tcp:close(Socket)

    end.


% wait_for_command(Socket, RobotId).


wait_for_command(Socket, RobotId) ->
    case gen_tcp:recv(Socket, 0) of
        {ok, Data} ->
            Command = binary_to_term(Data),

            io:format(
              "Robot ~p received command: ~p~n",
              [RobotId, Command]),

            handle_command(Socket, RobotId, Command),
            wait_for_command(Socket, RobotId);

        {error, closed} ->
            io:format(
              "Robot ~p disconnected~n",
              [RobotId]),
            gen_tcp:close(Socket)
    end.


handle_command(Socket, RobotId, {go, Goal, Positions}) ->
    io:format(
      "Robot ~p assigned goal ~p~n",
      [RobotId, Goal]),

    move_window(Socket, RobotId, Goal, Positions);

handle_command(Socket, RobotId, {wait, Reason}) ->
    io:format(
      "Robot ~p waiting: ~p~n",
      [RobotId, Reason]),

    gen_tcp:send(
      Socket,
      term_to_binary({wait_ack, RobotId}));

handle_command(Socket, RobotId, Command) ->
    io:format(
      "Robot ~p received unknown command: ~p~n",
      [RobotId, Command]),

    gen_tcp:send(
      Socket,
      term_to_binary({error, {unknown_command, Command}})).


move_window(Socket, RobotId, []) ->
    gen_tcp:send(
      Socket,
      term_to_binary({window_complete, RobotId}));

move_window(Socket, RobotId, [Position | Rest]) ->
    io:format(
      "Robot ~p moving to ~p~n",
      [RobotId, Position]),

    gen_tcp:send(
      Socket,
      term_to_binary({moved, RobotId, Position})),

    {ok, Data} = gen_tcp:recv(Socket, 0),
    Response = binary_to_term(Data),

    io:format(
      "Robot ~p received: ~p~n",
      [RobotId, Response]),

    case Response of
        {move_ack, Position} ->
            move_window(Socket, RobotId, Rest);

        {goal_reached, Position} ->
            io:format(
              "Robot ~p reached its goal at ~p~n",
              [RobotId, Position]),
            ok;

        {move_denied, Reason} ->
            io:format(
              "Robot ~p move denied: ~p~n",
              [RobotId, Reason]),
            ok;

        Other ->
            io:format(
              "Robot ~p received unexpected response: ~p~n",
              [RobotId, Other]),
            ok
    end.

move_window(_Socket, _RobotId, _Goal, []) ->
    ok;

%% Last position in the window: send the move, and immediately also
%% ask for the next window -- without waiting for this move's ack
%% first. TCP keeps the order, so the server always sees `moved`
%% before `window_complete` and already knows our new position when
%% it plans the next window. That means the next window is usually
%% already waiting for us by the time we finish this move and loop
%% back to wait_for_command, instead of us stalling for a round trip.
move_window(Socket, RobotId, Goal, [Position]) ->
    io:format(
      "Robot ~p moving to ~p~n",
      [RobotId, Position]),

    gen_tcp:send(
      Socket,
      term_to_binary({moved, RobotId, Position})),

    gen_tcp:send(
      Socket,
      term_to_binary({window_complete, RobotId, Goal})),

    {ok, Data} = gen_tcp:recv(Socket, 0),
    Response = binary_to_term(Data),

    io:format(
      "Robot ~p received: ~p~n",
      [RobotId, Response]),

    case Response of
        {move_ack, Position} ->
            ok;

        {goal_reached, Position} ->
            io:format(
              "Robot ~p reached its goal at ~p~n",
              [RobotId, Position]),
            ok;

        {move_denied, Reason} ->
            io:format(
              "Robot ~p move denied: ~p~n",
              [RobotId, Reason]),
            ok;

        Other ->
            io:format(
              "Robot ~p received unexpected response: ~p~n",
              [RobotId, Other]),
            ok
    end;

move_window(Socket, RobotId, Goal, [Position | Rest]) ->
    io:format(
      "Robot ~p moving to ~p~n",
      [RobotId, Position]),

    gen_tcp:send(
      Socket,
      term_to_binary({moved, RobotId, Position})),

    {ok, Data} = gen_tcp:recv(Socket, 0),
    Response = binary_to_term(Data),

    io:format(
      "Robot ~p received: ~p~n",
      [RobotId, Response]),

    case Response of
        {move_ack, Position} ->
            move_window(Socket, RobotId, Goal, Rest);

        {goal_reached, Position} ->
            io:format(
              "Robot ~p reached its goal at ~p~n",
              [RobotId, Position]),
            ok;

        {move_denied, Reason} ->
            io:format(
              "Robot ~p move denied: ~p~n",
              [RobotId, Reason]),
            ok;

        Other ->
            io:format(
              "Robot ~p received unexpected response: ~p~n",
              [RobotId, Other]),
            ok
    end.
