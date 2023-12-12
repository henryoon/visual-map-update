
"use strict";

let GetRobotMode = require('./GetRobotMode.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let AddToLog = require('./AddToLog.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetProgramState = require('./GetProgramState.js')
let Load = require('./Load.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let Popup = require('./Popup.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let RawRequest = require('./RawRequest.js')

module.exports = {
  GetRobotMode: GetRobotMode,
  IsInRemoteControl: IsInRemoteControl,
  AddToLog: AddToLog,
  GetSafetyMode: GetSafetyMode,
  GetLoadedProgram: GetLoadedProgram,
  GetProgramState: GetProgramState,
  Load: Load,
  IsProgramRunning: IsProgramRunning,
  Popup: Popup,
  IsProgramSaved: IsProgramSaved,
  RawRequest: RawRequest,
};
