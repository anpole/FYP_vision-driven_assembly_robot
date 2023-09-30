using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.IOSystemDomain;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using System;
using System.Drawing;

namespace RobotStudioEmptyAddin1
{
    public class Class1
    {
        // Global Variables
        static Part box = new Part();
        static Part prism = new Part();
        static Part cylinder = new Part();
        static Part bridge = new Part();
        static Part tprism = new Part();

        static Matrix4 boxM = new Matrix4(new Vector3(0, 0, 0), new Quaternion(Vector3.XVector, 0));
        static Matrix4 prismM = new Matrix4(new Vector3(0, 0, 0), new Quaternion(Vector3.XVector, 0));
        static Matrix4 cylinderM = new Matrix4(new Vector3(0, 0, 0), new Quaternion(Vector3.XVector, 0));
        static Matrix4 bridgeM = new Matrix4(new Vector3(0, 0, 0), new Quaternion(Vector3.XVector, 0));
        static Matrix4 tprismM = new Matrix4(new Vector3(0, 0, 0), new Quaternion(Vector3.XVector, 0));
        static Matrix4 translationMatrix = new Matrix4(new Vector3(0, 0, 0), new Quaternion(Vector3.XVector, 0));

        private static Controller _Vcontroller;
        private static Controller _Pcontroller;

        private static Task _task;
        private static Module _module;

        // Modifying Local Origin Functions
        private static Transform GetRefCoordSysTransforms(Part part)
        {
            Transform refTrf = null;

            //Get Parent of part object and store in base type(ProjectObject) variable.
            ProjectObject poPartParent = part.Parent;

            if (poPartParent != null && poPartParent.Parent is IHasTransform)
            {
                IHasTransform parent = poPartParent.Parent as IHasTransform;
                refTrf = parent.Transform;
            }
            else
                refTrf = null;

            return refTrf;
        }
        private static void ApplyLocal(Part part, Vector3 position, Vector3 orientation)
        {
            Project.UndoContext.BeginUndoStep();
            try
            {
                //Checks if Part variable refers to the instance Part object
                if (part != null)
                {
                    // Save old matrix for the part 
                    Matrix4 oldGlobalMatrix = part.Transform.GlobalMatrix;

                    //Get reference coordinate system transform value
                    Transform refTrf = GetRefCoordSysTransforms(part);

                    //Create new Identity matrix and apply position argument to matrix translation and 
                    //orientation argument to the matrix EulerZYX
                    Matrix4 mat = Matrix4.Identity;
                    mat.Translation = position;
                    mat.EulerZYX = orientation;

                    part.Transform.GlobalMatrix = (refTrf == null) ? mat : (refTrf.GlobalMatrix * mat);

                    // New matrix for the part       
                    Matrix4 newGlobalMatrix = part.Transform.GlobalMatrix;

                    // Calculate difference for moving back bodies  
                    newGlobalMatrix.InvertRigid();
                    Matrix4 diffTrans = newGlobalMatrix * oldGlobalMatrix;

                    // Move all the bodies back
                    if (part.HasGeometry)
                    {
                        foreach (Body bd in part.Bodies)
                        {
                            bd.Transform.Matrix = diffTrans * bd.Transform.Matrix;
                        }
                    }
                    else
                    {
                        // CQ5356
                        part.Mesh.Transform(diffTrans);
                        part.Mesh.Rebuild();
                    }
                }
            }
            catch
            {
                Project.UndoContext.CancelUndoStep(CancelUndoStepType.Rollback);
            }
            finally
            {
                Project.UndoContext.EndUndoStep();
            }
        }

        // String to matrix function


        private static Matrix4 CreateMatrixFromString(string input)
        {
            // Remove any whitespace and brackets from the string
            string cleanedInput = input.Replace(" ", "").Trim('[', ']');

            // Split the string into individual vector strings
            string[] vectorStrings = cleanedInput.Split(new[] { "],[", "], [", "],[ " }, StringSplitOptions.RemoveEmptyEntries);

            // Ensure we have exactly four vectors
            if (vectorStrings.Length != 2)
            {
                throw new ArgumentException("Invalid string format. The string should contain four vectors.");
            }

            // Parse each vector string and create Vector4 objects
            Vector3 vector1 = ParseVector3FromString(vectorStrings[0]);
            Quaternion quaternion = ParseQuaternionFromString(vectorStrings[1]);

            // Create the Matrix4 object using the four vectors
            Matrix4 matrix = new Matrix4(vector1, quaternion);

            return matrix;
        }
        private static Vector3 ParseVector3FromString(string vectorString)
        {
            // Remove brackets and split the string by commas
            string[] values = vectorString.Trim('[', ']').Split(',');

            if (values.Length != 4)
            {
                throw new ArgumentException("Invalid vector format. The vector should contain four values.");
            }

            // Parse the values and create a Vector4 object
            double x = double.Parse(values[1]);
            double y = double.Parse(values[2]);
            double z = double.Parse(values[3]);
            Vector3 vector = new Vector3(x, y, z);

            return vector;
        }
        private static Quaternion ParseQuaternionFromString(string vectorString)
        {
            // Remove brackets and split the string by commas
            string[] values = vectorString.Trim('[', ']').Split(',');

            if (values.Length != 4)
            {
                throw new ArgumentException("Invalid vector format. The vector should contain four values.");
            }

            // Parse the values and create a Vector4 object
            double x = double.Parse(values[0]);
            double y = double.Parse(values[1]);
            double z = double.Parse(values[2]);
            double w = double.Parse(values[3]);
            Quaternion quaternion = new Quaternion(x, y, z, w);

            return quaternion;
        }

        static void RequestMastership(Controller controller)
        {
            IMastershipResourceController mastershipController = controller as IMastershipResourceController;
            if (mastershipController != null)
            {
                // Request mastership control
                mastershipController.Request();
                // Perform operations that require mastership control
            }
            bool isMastershipGranted = controller.IsMaster;
            if (isMastershipGranted)
            {
                // Mastership acquired
                Logger.AddMessage(new LogMessage("Mastership granted.", "Add-In"));
                // Perform the desired operations on the controller
            }
            else
            {
                // Mastership not granted
                Logger.AddMessage(new LogMessage("Failed to acquire mastership. Another user may already have control.", "Add-In"));
            }
        }

        static void ReleaseMastership(Controller controller)
        {
            IMastershipResourceController mastershipController = controller as IMastershipResourceController;
            if (mastershipController != null)
            {
                // Request mastership control
                mastershipController.Release();
                // Perform operations that require mastership control
            }
            bool isMastershipGranted = controller.IsMaster;
            if (isMastershipGranted == false)
            {
                // Mastership acquired
                Logger.AddMessage(new LogMessage("Mastership released.", "Add-In"));
                // Perform the desired operations on the controller
            }

        }

        static void Signal_IOValueChanged(object sender, SignalChangedEventArgs e)
        {
            Station activeStation = Station.ActiveStation;
            RsWorkObject workObj = activeStation.ActiveTask.ActiveWorkObject;

            // Obtain the information about the signal change
            DigitalSignal signal = (DigitalSignal)sender;
            string signalName = signal.Name;
            object signalValue = signal.Value;
            string[] part = signalName.Split('_');

            // Do the desired actions due to the signal changing

            // Same Value in both Controller for Project Signals
            //if (part[0] == "APAV")
            //    {
            //        _Pcontroller.IOSystem.GetSignal(signalName).Value = _Vcontroller.IOSystem.GetSignal(signalName).Value;
            //    }


            if (signalName == "APAV_defineWorkObj" && signal.Value == 1)
            {
                RequestMastership(_Vcontroller);
                RequestMastership(_Pcontroller);

                workObj.UserFrame.Matrix = new Matrix4(Matrix3.Identity,new Vector3());
                activeStation.ActiveTask.ActiveWorkObject = workObj;

                RobTarget target1 = new RobTarget();
                RobTarget target2 = new RobTarget();
                RobTarget target3 = new RobTarget();
                
                string p;

                // Get the 3 targets
                p = _Pcontroller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData("TABLE_1").StringValue;
                _Vcontroller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData("TABLE_1").StringValue = p;
                target1.FillFromString2(p);
                p = _Pcontroller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData("TABLE_2").StringValue;
                _Vcontroller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData("TABLE_2").StringValue = p;
                target2.FillFromString2(p);
                p = _Pcontroller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData("TABLE_3").StringValue;
                _Vcontroller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData("TABLE_3").StringValue = p;
                target3.FillFromString2(p);

                // Calculate the WorkObject Translation
                Vector3 traslationVector =0.001 * new Vector3(target1.Trans.X, target1.Trans.Y, target1.Trans.Z);

                // Calculate the WorkObject Orientation
                Vector3 displacement2 = 0.001 * new Vector3(target2.Trans.X - target1.Trans.X, target2.Trans.Y - target1.Trans.Y, target2.Trans.Z - target1.Trans.Z);
                Vector3 displacement3 = 0.001 * new Vector3(target3.Trans.X - target1.Trans.X, target3.Trans.Y - target1.Trans.Y, target3.Trans.Z - target1.Trans.Z);

                Matrix3 rotationMatrix = new Matrix3(displacement2, displacement3, displacement2.Cross(displacement3));

                // Modify the WorkObject in Station
                Matrix4 wobjMatrix = new Matrix4(rotationMatrix, traslationVector);

                workObj.UserFrame.Matrix = wobjMatrix;
                activeStation.ActiveTask.ActiveWorkObject = workObj;

                // Obtein Quaternions
                Quaternion q = new Quaternion(new Vector3(workObj.UserFrame.RZ, workObj.UserFrame.RY, workObj.UserFrame.RX));

                // Modify the WorkObject in Controllers from the midpoint
                //Logger.AddMessage(new LogMessage(_Vcontroller.Rapid.GetTask("T_ROB1").GetModule("CalibData").GetRapidData("Workobject_1").StringValue, "Add-In"));

                _Vcontroller.Rapid.GetTask("T_ROB1").GetModule("CalibData").GetRapidData("Workobject_1").StringValue =
                    "[FALSE,TRUE,\"\",[[" + Convert.ToString(1000*workObj.UserFrame.X) + "," + Convert.ToString(1000 * workObj.UserFrame.Y) + "," + Convert.ToString(1000 * workObj.UserFrame.Z) + "]," +
                    "[" + q.q1 + "," + q.q2 + "," + q.q3 + "," + q.q4 + "]],[[0,0,0],[1,0,0,0]]]";
                _Pcontroller.Rapid.GetTask("T_ROB1").GetModule("CalibData").GetRapidData("Workobject_1").StringValue =
                    "[FALSE,TRUE,\"\",[[" + Convert.ToString(1000 * workObj.UserFrame.X) + "," + Convert.ToString(1000 * workObj.UserFrame.Y) + "," + Convert.ToString(1000 * workObj.UserFrame.Z) + "]," +
                    "[" + q.q1 + "," + q.q2 + "," + q.q3 + "," + q.q4 + "]],[[0,0,0],[1,0,0,0]]]";

                Logger.AddMessage(new LogMessage("WorkObject Updated at: " + _Vcontroller.Rapid.GetTask("T_ROB1").GetModule("CalibData").GetRapidData("Workobject_1").StringValue, "Add-In"));
                _Vcontroller.IOSystem.GetSignal("APAV_defineWorkObj").Value = 0;
                _Pcontroller.IOSystem.GetSignal("APAV_defineWorkObj").Value = 0;

                ReleaseMastership(_Vcontroller);
                ReleaseMastership(_Pcontroller);
            }

            if (signal.Name == "APAV_createPieces" && signal.Value == 1)
            {
                _Vcontroller.IOSystem.GetSignal("APAV_createPieces").Value = 0;

                // Create a part to contain each body.          

                box.Name = "p1";
                activeStation.GraphicComponents.Add(box);

                prism.Name = "p2";
                activeStation.GraphicComponents.Add(prism);


                cylinder.Name = "p3";
                activeStation.GraphicComponents.Add(cylinder);


                bridge.Name = "p4";
                activeStation.GraphicComponents.Add(bridge);


                tprism.Name = "p5";
                activeStation.GraphicComponents.Add(tprism);


                // Create a box.
                Matrix4 matrix_origo = new Matrix4(new Vector3(Axis.X), 0.0);
                Vector3 box_size = new Vector3(0.02, 0.02, 0.02);
                Body b1 = Body.CreateSolidBox(matrix_origo, box_size);
                b1.Name = "Box";
                box.Bodies.Add(b1);
                b1.Color = Color.Yellow;

                ApplyLocal(box, 0.001 * new Vector3(10, 10, 0), new Vector3(0, 0, 0));
                box.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, translationMatrix);

                // Create a square prism.
                Vector3 prism_size = new Vector3(0.05, 0.02, 0.02);
                Body b2 = Body.CreateSolidBox(matrix_origo, prism_size);
                b2.Name = "Square Prism";
                prism.Bodies.Add(b2);
                b2.Color = Color.Blue;

                ApplyLocal(prism, 0.001 * new Vector3(25, 10, 0), new Vector3(0, 0, 0));
                prism.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, translationMatrix);


                // Create a cylinder.
                Body b3 = Body.CreateSolidCylinder(matrix_origo, 0.01, 0.02);
                b3.Name = "Cylinder";
                cylinder.Bodies.Add(b3);
                b3.Color = Color.Red;

                cylinder.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, translationMatrix);

                // Create a triangular prism.
                Vector3 b4_size = new Vector3(0.02, 0.02, 0.02);
                Vector4 b4_x = new Vector4(0.5, 0.866, 0, 0);
                Vector4 b4_y = new Vector4(-0.866, 0.5, 0, 0);
                Vector4 b4_z = new Vector4(0, 0, 1, 0);
                Vector4 b4_t = new Vector4(0, 0, 0, 1);
                Matrix4 b4_matrix = new Matrix4(b4_x, b4_y, b4_z, b4_t);

                Vector4 b5_x = new Vector4(0.8660, 0.5, 0, 0);
                Vector4 b5_y = new Vector4(-0.5, 0.866, 0, 0);
                Vector4 b5_z = new Vector4(0, 0, 1, 0);
                Vector4 b5_t = new Vector4(0.02, 0, 0, 1);
                Matrix4 b5_matrix = new Matrix4(b5_x, b5_y, b5_z, b5_t);

                Vector3 b6_size = new Vector3(0.02, 0.01732, 0.02);

                Body b4 = Body.CreateSolidBox(b4_matrix, b4_size);
                Body b5 = Body.CreateSolidBox(b5_matrix, b4_size);
                Body b6 = Body.CreateSolidBox(matrix_origo, b6_size);

                Body[] cut1 = b6.Cut2(b4);
                foreach (Body b6_1 in cut1)
                {
                    Body[] cut2 = b6_1.Cut2(b5);
                    foreach (Body b6_2 in cut2)
                    {
                        b6_2.Name = "Triangular Prism";
                        tprism.Bodies.Add(b6_2);
                        b6_2.Color = Color.MediumPurple;
                    }
                }

                ApplyLocal(tprism, 0.001 * new Vector3(10, 8.66, 0), new Vector3(0, 0, 0));
                tprism.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, translationMatrix);


                // Create a bridge. 

                Vector4 b7_x = new Vector4(1, 0, 0, 0);
                Vector4 b7_y = new Vector4(0, 1, 0, 0);
                Vector4 b7_z = new Vector4(0, 0, 0, 1);
                Vector4 b7_t = new Vector4(0.025, 0.02, 0, 1);
                Matrix4 b7_matrix = new Matrix4(b7_x, b7_y, b7_z, b7_t);

                Body b7 = Body.CreateSolidBox(matrix_origo, prism_size);
                Body b8 = Body.CreateSolidCylinder(b7_matrix, 0.0163, 0.02);

                Body[] cut3 = b7.Cut2(b8);
                foreach (Body b9 in cut3)
                {
                    b9.Name = "Bridge";
                    bridge.Bodies.Add(b9);
                    b9.Color = Color.Green;
                }

                ApplyLocal(bridge, 0.001 * new Vector3(4.35, 10, 0), new Vector3(0, 0, 0));
                bridge.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, translationMatrix);

                Logger.AddMessage(new LogMessage("Pieces Created.", "Add-In"));
                _Vcontroller.IOSystem.GetSignal("APAV_piecesCreated").Value = 1;
                _Pcontroller.IOSystem.GetSignal("APAV_piecesCreated").Value = 1;
            }

            if (signal.Name == "APAV_updatePieces" && signal.Value == 1)
            {

                // Represent the presence of the Pieces in the Workspace
                if (_Vcontroller.IOSystem.GetSignal("APAV_piece_1").Value == 0) { activeStation.GraphicComponents.Remove(box); box.Bodies.Clear(); }
                else
                {
                    // Get the new Positions of the Pieces
                    boxM = CreateMatrixFromString(_module.GetRapidData("cubeM").StringValue);
                    // Update the new Positions
                    box.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, boxM);
                }
                if (_Vcontroller.IOSystem.GetSignal("APAV_piece_2").Value == 0) { activeStation.GraphicComponents.Remove(prism); prism.Bodies.Clear(); }
                else
                {
                    // Get the new Positions of the Pieces
                    prismM = CreateMatrixFromString(_module.GetRapidData("prismM").StringValue);
                    // Update the new Positions
                    prism.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, prismM);
                }

                if (_Vcontroller.IOSystem.GetSignal("APAV_piece_3").Value == 0) { activeStation.GraphicComponents.Remove(cylinder); cylinder.Bodies.Clear(); }
                else
                {
                    // Get the new Positions of the Pieces
                    cylinderM = CreateMatrixFromString(_module.GetRapidData("cylinderM").StringValue);
                    // Update the new Positions
                    cylinder.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, cylinderM);

                }
                if (_Vcontroller.IOSystem.GetSignal("APAV_piece_4").Value == 0) { activeStation.GraphicComponents.Remove(bridge); bridge.Bodies.Clear(); }
                else
                {
                    // Get the new Positions of the Pieces
                    bridgeM = CreateMatrixFromString(_module.GetRapidData("bridgeM").StringValue);
                    // Update the new Positions
                    bridge.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, bridgeM);
                }
                if (_Vcontroller.IOSystem.GetSignal("APAV_piece_5").Value == 0) { activeStation.GraphicComponents.Remove(tprism); tprism.Bodies.Clear(); }
                else
                {
                    // Get the new Positions of the Pieces
                    tprismM = CreateMatrixFromString(_module.GetRapidData("tprismM").StringValue);
                    // Update the new Positions
                    tprism.Transform.SetRelativeTransform(workObj.UserFrame.GlobalMatrix, tprismM);
                }
                Logger.AddMessage(new LogMessage("Pieces Updated.", "Add-In"));
            }

            if (signal.Name == "APAV_deletePieces" && signal.Value == 1)
            {
                _Vcontroller.IOSystem.GetSignal("APAV_piecesCreated").Value = 0;
                _Vcontroller.IOSystem.GetSignal("APAV_deletePieces").Value = 0;

                _Pcontroller.IOSystem.GetSignal("APAV_piecesCreated").Value = 0;
                _Pcontroller.IOSystem.GetSignal("APAV_deletePieces").Value = 0;

                // Delete parts in the station
                ApplyLocal(box,new  Vector3(0, 0, 0), new Vector3(0, 0, 0));
                activeStation.GraphicComponents.Remove(box);
                box.Bodies.Clear();
                ApplyLocal(cylinder, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
                activeStation.GraphicComponents.Remove(cylinder);
                cylinder.Bodies.Clear();
                ApplyLocal(prism, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
                activeStation.GraphicComponents.Remove(prism);
                prism.Bodies.Clear();
                ApplyLocal(bridge, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
                activeStation.GraphicComponents.Remove(bridge);
                bridge.Bodies.Clear();
                ApplyLocal(tprism, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
                activeStation.GraphicComponents.Remove(tprism);
                tprism.Bodies.Clear();
            }


        }

        // This is the entry point which will be called when the Add-in is loaded

        public static void AddinMain()
        {
            Project.UndoContext.BeginUndoStep("Logger");
            try
            {
                // Add category.
                if (!Logger.CategoryCaptions.ContainsKey("Add-In"))
                {
                    Logger.CategoryCaptions.Add("Add-In", "Add-In");
                }

                // Add message for correct connection.
                Logger.AddMessage(new LogMessage("This is just a test message to prove that the Add-in is working!", "Add-In"));
            }
            catch
            {
                Project.UndoContext.CancelUndoStep(CancelUndoStepType.Rollback);
                throw;
            }
            finally
            {
                Project.UndoContext.EndUndoStep();
            }

            // Controller Connection
            NetworkScanner networkScanner = new NetworkScanner();
            ControllerInfo[] controllers = networkScanner.GetControllers();
            foreach (ControllerInfo control in controllers)
            {
                Logger.AddMessage(new LogMessage("Controller: " + control.Name, "Add-In"));
            }
            for (int i = 0; i < controllers.Length; i++)
            {
                if (controllers[i].Name == "FYP_v00")
                {
                    _Vcontroller = ControllerFactory.CreateFrom(controllers[i]);
                    _Vcontroller.Logon(UserInfo.DefaultUser);
                    // Verify if the conection is correct
                    if (_Vcontroller.Connected)
                    {
                        Logger.AddMessage(new LogMessage("Virtual Controller Conected: " + controllers[i], "Add-In"));
                        _task = _Vcontroller.Rapid.GetTask("T_ROB1");
                        _module = _task.GetModule("MainModule");
                    }

                }
                if (controllers[i].Name == "15000-100009")
                {
                    _Pcontroller = ControllerFactory.CreateFrom(controllers[i]);
                    _Pcontroller.Logon(UserInfo.DefaultUser);
                    // Verify if the conection is correct
                    if (_Pcontroller.Connected)
                    {
                        Logger.AddMessage(new LogMessage("Physical Controller Conected: " + controllers[i], "Add-In"));
                    }
                }
            }

            foreach (Signal signal in _Vcontroller.IOSystem.GetSignals(IOFilterTypes.Digital))
            {
                signal.Changed += Signal_IOValueChanged;
            }
        }
    }
}
