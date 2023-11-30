
using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Mars_Rover_Configuration;
using Mars_Rover_Comms;
using System.Xml.Serialization;
using Mars_Rover_RCU;
using Mars_Rover_RCU.Comms;
using Mars_Rover_RCU.Controllers;
using Pololu.UsbWrapper;
using Mars_Rover_RCU.Utilities;

namespace Mars_Rover_RCU
{
    //Main Program Entry PoC:\Users\Jason\Dropbox\My Documents\WVU\Robotics_2014\Software\Mars_Rover_Master\Mars_Rover_RCU\Program.csint
    public class Program
    {
        private static RCUComms comms;

        static bool debug = true;

        static ConfigureRCU rcuConfig; //Responsible for sending states back to OCU
        static StreamWriter log;

        static public Controllers.PhidgetsController _Phidgets;
        static Controllers.Maestro _Maestro;
        static Controllers.Arm _Arm;// = new Controllers.Arm();//(rcuConfig.ArmConfig);
        static public Controllers.GPS _GPS;
        static Controllers.Roomba _Roomba = new Roomba();

        static Utility.UpdateQueue<RobotState> stateQueue = new Utility.UpdateQueue<RobotState>(-1);
        static XmlSerializer robotStateDeserializer = new XmlSerializer(typeof(Mars_Rover_Comms.RobotState));

        //tcp/ip client for communicating with the ocu
        static public Mars_Rover_Comms.TCP.ZClient client;

        static Boolean useMaestro = true;
        static Boolean useGPS = true;
        static Boolean useArm = true;
        static Boolean usePhidgets = true;
        // static Boolean useRoomba = true;
        //static public bool connected;

        static Thread stateProcessor;
        static CancellationTokenSource tokenSource = new CancellationTokenSource();

        static Mars_Rover_RCU.Kinematics.Kinematics kinematics;

        static DateTime APMconnectTime = DateTime.Now;

        public static void Main(string[] args)
        {

            //setup primary comms
            //client = new Mars_Rover_Comms.TCP.ZClient("157.182.34.76", 1111); //Server Room
            //client = new Mars_Rover_Comms.TCP.ZClient("127.0.0.1", 1111);  //local host: 127.0.0.1
            // client = new Mars_Rover_Comms.TCP.ZClient("157.182.211.157", 1111); 
            //client = new Mars_Rover_Comms.TCP.ZClient("157.182.34.76",1111);
            client = new Mars_Rover_Comms.TCP.ZClient("157.182.34.110", 1111);

            client.PacketReceived += new EventHandler<DataArgs>(client_PacketReceived);

            foreach (string str in args)
            {


                char[] separator = { '=' };
                string[] arg_pair = str.Split(separator, 2, StringSplitOptions.RemoveEmptyEntries);

                if (str == "--no_maestro")
                {
                    Logger.WriteLine("No Maestro Specified!");
                    useMaestro = false;
                }
                else if (str == "--no_gps")
                {
                    Logger.WriteLine("No GPS Specified!");
                    useGPS = false;
                }
                else if (str == "--no_arm")
                {
                    Logger.WriteLine("No ARM Specified!");
                    useArm = false;
                }
                else if (str == "--no_phidgets")
                {
                    Logger.WriteLine("No Phidgets Specified");
                    usePhidgets = false;
                }
                /* else if (str == "--no_roomba")
                 {
                     Logger.WriteLine("No Roomba Specified");
                     useRoomba = false;
                 }*/

                 /*  else
                   {
                       // handle the config file option
                       switch (arg_pair[0])
                       {
                           case "--config":
                               //load file...
                               XmlSerializer deserializer = new System.Xml.Serialization.XmlSerializer(typeof(ConfigureRCU));
                               try
                               {
                                   using (StreamReader file_reader = new StreamReader(arg_pair[1]))
                                   {
                                       rcuConfig = (ConfigureRCU)deserializer.Deserialize(file_reader);
                                   }
                               }
                               catch (Exception ex)
                               {
                                   Logger.WriteLine("Error opening configuration file: " + ex.Message);
                               }
                               break;
                           default:
                               break;
                       }
                   }*/
                 
            }

            /* try
             {
                 if (rcuConfig == null)
                 {
                     Logger.WriteLine("Usage: mars.rcu.exe --config=filename [--no_arm] [--no_apm] [--no_maestro]");
                     throw new Exception("No configuration specified");
                 }
                 */ 
               comms = RCUComms.Instance; //Responsible for sending states back to OCU
            /*
              // Logger.WriteLine("made it");
              #region Phidgets
               if (usePhidgets)
                {
                 _Phidgets = new PhidgetsController();
                 Logger.WriteLine("Initialized Phidgets");
                }*/
            //#endregion
            // Logger.WriteLine("made it");
            #region Roomba
            //if (useRoomba)
            //_Roomba = new Controllers.Roomba();

            #endregion

            # region GPS
            /* if (useGPS)
                {
                    _GPS = new GPS();
                    _GPS.startPort();
                }*/
            #endregion

            #region Maestro
               if (useMaestro)
                {
                 //   _Maestro = new Controllers.Maestro(rcuConfig.MaestroConfig);
                   
                   // kinematics = new Kinematics.Kinematics(1000);
                }
                
            #endregion

            #region Arm
            if (useArm)
            {
               // _Arm = new Controllers.Arm(rcuConfig.ArmConfig);
               // _Arm = new Controllers.Arm();
                //_Arm.ConnectArm(5);
                //_Arm.InitializeServos();
            }
            #endregion
            //StateProcessorDoWork();
            // packet handler - runs in its own thread
            stateProcessor = new Thread(new ThreadStart(StateProcessorDoWork));
            stateProcessor.Name = "State Processor";
            //Logger.WriteLine("Made it");
            stateProcessor.Start();
            //


            /* }
             catch (Exception ex)
             {
                 Logger.WriteLine("Error during startup: " + ex.Message);
                 //return;
             }*/

            while (true)
            {
                Logger.WriteLine("Enter exit to shutdown");
                string input = Console.ReadLine().ToLower();
                if (input.Contains("exit"))
                {
                    break;
                }
            }

            //Closing Remarks
            client.PacketReceived -= client_PacketReceived;
            client.Close();
            tokenSource.Cancel();
            //if (stateProcessor != null)
            //stateProcessor.Join();

            if (useMaestro && _Maestro != null)
                _Maestro.Deactivate();

            if (useArm) // Add check that arm object is not null
            {
                //Disconnect Arm
                _Arm.DisconnectArm();
                Logger.WriteLine("Disconnected the arm.");
            }

            if (usePhidgets && _Phidgets != null)
            {
                _Phidgets.lossOfSignalOff();
                _Phidgets.closePhidgetsController();
            }
        }
        //End Main

        // handler for commands received from the ocu
        static void client_PacketReceived(object sender, DataArgs e)
        {
            try
            {
                using (MemoryStream ms = new MemoryStream(e.Data))
                {
                    // robot drive state received - enqueue the state so it is processed in the StateProcessorDoWork() below
                    RobotState state = (RobotState)robotStateDeserializer.Deserialize(ms);
                    stateQueue.Enqueue(state);
                }
            }
            catch (Exception ex)
            {
                Logger.WriteLine("Error deserializing state: " + ex.Message);
            }
        }

        static void StateProcessorDoWork()
        {

            while (!tokenSource.Token.IsCancellationRequested)
            {
                //connected = client.IsConnected();


                try
                {
                    Mars_Rover_Comms.RobotState robotState = stateQueue.Dequeue(tokenSource.Token);

                    // Logger.WriteLine("");

                    if (client.IsConnected())
                    {

                        _Roomba.LOS(false);
                        //_Phidgets.lossOfSignalOff();
                        // Logger.WriteLine("");
                        if (/*(usePhidgets) &&*/ (robotState.DriveState != null))
                        {
                            //  Logger.WriteLine(" ");
                            if (robotState.DriveState.Headlights == true)
                            {
                                // _Phidgets.turnOnLights();
                                _Roomba.lights(true);
                            }
                            else if (robotState.DriveState.Headlights == false)
                            {
                                //_Phidgets.turnOffLights();
                                _Roomba.lights(false);
                            }
                        }

                        if ((useMaestro) && (robotState.DriveState != null))
                        {
                            // Dictionary<Devices, int> driveState = kinematics.GetWheelStates(robotState.DriveState.Radius, robotState.DriveState.Speed, /*robotState.DriveState.ArmSpeed*/ 0, /*robotState.DriveState.ScoopIn*/ false, /*robotState.DriveState.ScoopOut*/false, /*robotState.DriveState.FrontStopArmUp*/ false, /*robotState.DriveState.FrontStopArmDown*/false, /*robotState.DriveState.WallFollow*/ false);
                            //driveState[Devices.ControlSignal] = robotState.DriveState.Control;
                            //_Maestro.EnqueueState(driveState);
                            _Roomba.drive(robotState);
                        }

                        if ((useArm) && (robotState.DriveState.WallFollow))
                        {
                            // int state = robotState.ArmState.ArmData;
                            //_Arm.EnqueueState(state);
                            _Roomba.launch();

                        }
                        else
                        {
                            _Roomba.setArm();
                        }
                        if((useArm) && !(robotState.DriveState.Headlights))
                        {
                            _Roomba.close();
                        }
                        if((useArm) && (robotState.DriveState.Headlights)){
                            _Roomba.open();
                        }
                        if ((useArm) && (robotState.DriveState.FrontStopArmDown))
                        {
                            _Roomba.down();
                        }
                        if ((useArm) && (robotState.DriveState.FrontStopArmUp))
                        {
                            _Roomba.up();
                        }

                        /*  if ((robotState.CmdLineState != null))
                          {
                              if (robotState.CmdLineState.CmdLine.Equals("welcome houston"))
                                  Logger.WriteLine("bazingaaa!");

                          }*/
                    }
                    else
                    {
                        // _Phidgets.lossOfSignalOn();
                        _Roomba.LOS(true);

                        if (useMaestro)
                        {
                            Dictionary<Devices, int> driveState = kinematics.GetWheelStates(2047, 0, 0, false, false, false, false, false);
                            driveState[Devices.ControlSignal] = 0;
                            _Maestro.EnqueueState(driveState);
                        }
                    }
                }
                catch (OperationCanceledException ex)
                {
                    Logger.WriteLine("State Processor: " + ex.Message);
                    break;
                }
                catch (Exception ex)
                {
                    // Logger.WriteLine("StateProcessor: unhandled exception: " + ex.Message);
                }
            }
            Logger.WriteLine("StateProcessor exiting...");



        }

    }
}
