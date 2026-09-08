%%%-------------------------------------------------------------------
%% @doc robo_server public API
%% @end
%%%-------------------------------------------------------------------

-module(robo_server_app).

-behaviour(application).

-export([start/2, stop/1]).

start(_StartType, _StartArgs) ->
    robo_server_sup:start_link().

stop(_State) ->
    ok.

%% internal functions
