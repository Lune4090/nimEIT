import std/[rdstdin]
import loop

type
  SystemMode* = enum
    Forward,
    Backward,
    Quit,
  
proc set_system_mode(): (SystemMode, bool) =
  ## bool: 新データを保存するかどうか
  while true:
    let mode_num = readLineFromStdin("Mode: ")
    if mode_num != "0" and mode_num != "1" and mode_num != "2" and mode_num != "3":
      echo "Input is invalid, please try again"
    if mode_num == "1":
      return (Forward, false)
    if mode_num == "2":
      return (Forward, true)
    if mode_num == "3":
      return (Backward, false)
    if mode_num == "0":
      return (Quit, false)

proc cli_navi*() =
  echo "Welcome to nimEIT! please choose the mode..."
  echo "1: forward-non-data-creation"
  echo "2: forward-data-creation"
  echo "3: backward"
  echo "0: exit"

  let
    (systemMode, preserveData) = set_system_mode()
    

  case systemMode
    of Forward:
      echo "Forward Mode"
      let
        meshName = readLineFromStdin("Mesh folder: ")
        settingFileName = readLineFromStdin("Setting file name: ")
      forward_loop(preserveData, meshName, settingFileName)

    of Backward:
      echo "Backward Mode"
      let
        meshName = readLineFromStdin("Mesh folder: ")
      backward_loop(meshName)
    
    of Quit:
      echo "Good bye!"