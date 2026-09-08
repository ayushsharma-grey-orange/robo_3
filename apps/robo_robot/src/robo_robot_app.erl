%%%-------------------------------------------------------------------
%% @doc robo_robot public API
%% @end
%%%-------------------------------------------------------------------

-module(robo_robot_app).

-behaviour(application).

-export([start/2, stop/1]).

start(_StartType, _StartArgs) ->
    robo_robot_sup:start_link().

stop(_State) ->
    ok.

%% internal functions
