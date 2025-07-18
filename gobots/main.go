package main

import (
	"context"
	"image/color"
	"time"

	"github.com/golang/geo/r3"
	"github.com/lucasb-eyer/go-colorful"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

const maximumIntensity = 255.0

func RGBAToHSV(rgba color.RGBA) (h, s, v float64) {
	r := float64(rgba.R) / maximumIntensity
	g := float64(rgba.G) / maximumIntensity
	b := float64(rgba.B) / maximumIntensity
	c := colorful.Color{R: r, G: g, B: b}
	return c.Hsv()
}

func main() {
	logger := logging.NewLogger("client")
	machine, err := client.New(
		context.Background(),
		"",
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			"",
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: "",
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}

	defer machine.Close(context.Background())
	logger.Info("Resources:")
	logger.Info(machine.ResourceNames())

	fsCfg, err := machine.FrameSystemConfig(context.Background())
	if err != nil {
		logger.Error(err)
		return
	}
	logger.Info(fsCfg)

	_, err = referenceframe.NewFrameSystem("pick_and_place", fsCfg.Parts, nil)
	if err != nil {
		logger.Error(err)
		return
	}

	// pc
	cam, err := camera.FromRobot(machine, "pc")
	if err != nil {
		logger.Error(err)
		return
	}
	pc, err := cam.NextPointCloud(context.Background())
	if err != nil {
		logger.Error(err)
		return
	}

	// transform to frame of reference of the camera
	pcw, err := machine.TransformPointCloud(context.Background(), pc, cam.Name().ShortName(), referenceframe.World)
	if err != nil {

	}

	// filter out points that are not green
	greenPoints := pointcloud.NewBasicPointCloud(0)
	pcw.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if d.HasColor() {
			r, g, b, a := d.Color().RGBA()
			h, s, v := RGBAToHSV(color.RGBA{uint8(r), uint8(g), uint8(b), uint8(a)})
			if h > 100 && h < 140 && s > 0.3 && v > 0.3 {
				greenPoints.Set(p, d)
			}
		}
		return true
	})

	findCentroid := func() r3.Vector {
		sum := r3.Vector{}
		count := 0
		greenPoints.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			sum = sum.Add(p)
			count++
			return true
		})
		return sum.Mul(1.0 / float64(count))
	}

	centroid := findCentroid()
	centroid.Add(r3.Vector{pcw.MetaData().MaxX, pcw.MetaData().MaxY, pcw.MetaData().MaxZ})
	logger.Infof("centroid plus max: %+v", centroid)

	// move the arm to the centroid
	pif := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPoseFromPoint(centroid))

	// armie
	armie, err := arm.FromRobot(machine, "armie")
	if err != nil {
		logger.Error(err)
		return
	}
	m, err := motion.FromRobot(machine, "builtin")
	if err != nil {
		logger.Error(err)
		return
	}

	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{{}},
	}

	req := motion.MoveReq{
		ComponentName: armie.Name(),
		Destination:   pif,
		Constraints:   constraints,
	}
	ok, err := m.Move(context.Background(), req)
	if err != nil {
		logger.Error(err)
		return
	}
	logger.Infof("move ok: %+v", ok)

	g, err := gripper.FromRobot(machine, "gripper")
	if err != nil {
		logger.Error(err)
		return
	}

	if err := g.Open(context.Background(), nil); err != nil {
		logger.Error(err)
		return
	}

	if err := g.Close(context.Background()); err != nil {
		logger.Error(err)
		return
	}

	// time.Sleep to wait for the gripper to close
	time.Sleep(1 * time.Second)

	logger.Info("moving to end position")
	endPif := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPoseFromPoint(r3.Vector{300, 250, 0.1}))
	ok, err = m.Move(context.Background(), motion.MoveReq{
		ComponentName: armie.Name(),
		Destination:   endPif,
		Constraints:   constraints,
	})
	if err != nil {
		logger.Error(err)
		return
	}
	logger.Infof("move 2 ok: %+v", ok)

	time.Sleep(1 * time.Second)

}
