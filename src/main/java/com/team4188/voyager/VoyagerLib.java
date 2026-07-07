package com.team4188.voyager;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Robot-side bridge between the Voyager autonomous app and BLine-Lib.
 *
 * <p>Default deploy layout:
 *
 * <pre>
 * src/main/deploy/voyager/auto_config.json
 * src/main/deploy/voyager/YourPathName.json
 * </pre>
 *
 * <p>The file intentionally avoids third-party JSON imports. It uses a small internal parser so
 * robot projects do not need any third-party JSON parser dependency.
 */
public final class VoyagerLib {
  private static final String SELECTED_AUTO_TOPIC = "/Voyager/SelectedAuto";

  private static final Map<String, BooleanSupplier> CONDITIONS = new ConcurrentHashMap<>();
  private static final Map<String, Runnable> RUNNABLE_EVENTS = new ConcurrentHashMap<>();
  private static final Map<String, Command> COMMAND_EVENTS = new ConcurrentHashMap<>();
  private static final Map<String, Integer> LOOP_REPETITIONS = new ConcurrentHashMap<>();
  private static final Set<String> WARNED_MISSING_CONDITIONS = ConcurrentHashMap.newKeySet();
  private static final Set<String> WARNED_MISSING_EVENTS = ConcurrentHashMap.newKeySet();

  private static Subsystem driveSubsystem;
  private static Supplier<Pose2d> poseSupplier;
  private static Consumer<Pose2d> resetPoseConsumer;
  private static Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private static Consumer<ChassisSpeeds> chassisSpeedsConsumer;
  private static boolean flipIfRed;

  private static Supplier<PIDController> translationControllerFactory =
      () -> new PIDController(5.0, 0.0, 0.0);
  private static Supplier<PIDController> rotationControllerFactory =
      () -> {
        PIDController controller = new PIDController(3.0, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
      };
  private static Supplier<PIDController> crossTrackControllerFactory =
      () -> new PIDController(2.0, 0.0, 0.0);

  private static Path.DefaultGlobalConstraints defaultGlobalConstraints =
      new Path.DefaultGlobalConstraints(4.5, 10.0, 720.0, 1500.0, 0.03, 2.0, 0.2);

  private static java.nio.file.Path voyagerDirectory = defaultVoyagerDirectory();
  private static StringSubscriber selectedAutoSubscriber;
  private static Consumer<Rotation2d> moduleOrientationConsumer;
  private static Method reflectedModuleOrientationMethod;

  private static final Object AUTO_LOCK = new Object();
  private static Map<String, AutoDefinition> autoDefinitions = Map.of();
  private static long autoConfigLastModifiedMillis = Long.MIN_VALUE;
  private static Command currentAutoCommand = Commands.none();
  private static String currentAutoId = "";

  private VoyagerLib() {}

  public static void configure(
      Subsystem drive,
      Supplier<Pose2d> getPose,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> getChassisSpeeds,
      Consumer<ChassisSpeeds> setChassisSpeeds,
      boolean flipIfRed) {
    driveSubsystem = Objects.requireNonNull(drive, "drive");
    poseSupplier = Objects.requireNonNull(getPose, "getPose");
    resetPoseConsumer = Objects.requireNonNull(resetPose, "resetPose");
    chassisSpeedsSupplier = Objects.requireNonNull(getChassisSpeeds, "getChassisSpeeds");
    chassisSpeedsConsumer = Objects.requireNonNull(setChassisSpeeds, "setChassisSpeeds");
    VoyagerLib.flipIfRed = flipIfRed;
    reflectedModuleOrientationMethod = findModuleOrientationMethod(driveSubsystem);

    Path.setDefaultGlobalConstraints(defaultGlobalConstraints);
    selectedAutoSubscriber =
        NetworkTableInstance.getDefault().getStringTopic(SELECTED_AUTO_TOPIC).subscribe("");
    reloadAutos(true);
  }

  public static void setAutosDirectory(java.nio.file.Path autosDir) {
    voyagerDirectory = Objects.requireNonNull(autosDir, "autosDir");
    autoConfigLastModifiedMillis = Long.MIN_VALUE;
    reloadAutos(true);
  }

  public static void setModuleOrientationConsumer(Consumer<Rotation2d> orientModules) {
    moduleOrientationConsumer = Objects.requireNonNull(orientModules, "orientModules");
  }

  public static void setDefaultGlobalConstraints(
      double maxVelocityMetersPerSec,
      double maxAccelerationMetersPerSec2,
      double maxVelocityDegPerSec,
      double maxAccelerationDegPerSec2,
      double endTranslationToleranceMeters,
      double endRotationToleranceDeg,
      double intermediateHandoffRadiusMeters) {
    defaultGlobalConstraints =
        new Path.DefaultGlobalConstraints(
            maxVelocityMetersPerSec,
            maxAccelerationMetersPerSec2,
            maxVelocityDegPerSec,
            maxAccelerationDegPerSec2,
            endTranslationToleranceMeters,
            endRotationToleranceDeg,
            intermediateHandoffRadiusMeters);
    Path.setDefaultGlobalConstraints(defaultGlobalConstraints);
  }

  public static void setFollowerGains(
      double translationP,
      double translationI,
      double translationD,
      double rotationP,
      double rotationI,
      double rotationD,
      double crossTrackP,
      double crossTrackI,
      double crossTrackD) {
    setPIDControllerFactories(
        () -> new PIDController(translationP, translationI, translationD),
        () -> {
          PIDController controller = new PIDController(rotationP, rotationI, rotationD);
          controller.enableContinuousInput(-Math.PI, Math.PI);
          return controller;
        },
        () -> new PIDController(crossTrackP, crossTrackI, crossTrackD));
  }

  public static void addPIDControllers(
      PIDController translationController,
      PIDController rotationController,
      PIDController crossTrackController) {
    setPIDControllers(translationController, rotationController, crossTrackController);
  }

  public static void setPIDControllers(
      PIDController translationController,
      PIDController rotationController,
      PIDController crossTrackController) {
    Objects.requireNonNull(translationController, "translationController");
    Objects.requireNonNull(rotationController, "rotationController");
    Objects.requireNonNull(crossTrackController, "crossTrackController");
    setPIDControllerFactories(
        () -> copyPidController(translationController, false),
        () -> copyPidController(rotationController, true),
        () -> copyPidController(crossTrackController, false));
  }

  public static void setPIDControllerFactories(
      Supplier<PIDController> translationControllerFactory,
      Supplier<PIDController> rotationControllerFactory,
      Supplier<PIDController> crossTrackControllerFactory) {
    VoyagerLib.translationControllerFactory = Objects.requireNonNull(translationControllerFactory);
    VoyagerLib.rotationControllerFactory = Objects.requireNonNull(rotationControllerFactory);
    VoyagerLib.crossTrackControllerFactory = Objects.requireNonNull(crossTrackControllerFactory);
  }

  public static void addConditional(String key, BooleanSupplier condition) {
    CONDITIONS.put(
        normalizeKey(key, "conditional key"), Objects.requireNonNull(condition, "condition"));
  }

  public static void addEvent(String key, Runnable runnable) {
    String normalized = normalizeKey(key, "event key");
    RUNNABLE_EVENTS.put(normalized, Objects.requireNonNull(runnable, "runnable"));
    COMMAND_EVENTS.remove(normalized);
    FollowPath.registerEventTrigger(normalized, runnable);
  }

  public static void addEvent(String key, Command command) {
    String normalized = normalizeKey(key, "event key");
    COMMAND_EVENTS.put(normalized, Objects.requireNonNull(command, "command"));
    RUNNABLE_EVENTS.remove(normalized);
    FollowPath.registerEventTrigger(normalized, command);
  }

  public static void addLoopRepetitions(String key, int repetitions) {
    if (repetitions < 0) {
      throw new IllegalArgumentException("Loop repetitions cannot be negative");
    }
    LOOP_REPETITIONS.put(normalizeKey(key, "loop key"), repetitions);
  }

  public static Command runSelectedAuto() {
    ensureConfigured();
    reloadAutos(false);

    String selectedId = selectedAutoSubscriber == null ? "" : selectedAutoSubscriber.get().trim();
    if (selectedId.isBlank()) {
      selectedId = firstAutoId();
    }

    AutoDefinition selected = autoDefinitions.get(selectedId);
    if (selected == null) {
      DriverStation.reportWarning(
          "Voyager selected auto '"
              + selectedId
              + "' was not found. Available autos: "
              + autoDefinitions.keySet(),
          false);
      currentAutoId = selectedId;
      currentAutoCommand = Commands.none();
      return currentAutoCommand;
    }

    currentAutoId = selected.id();
    currentAutoCommand = buildAutoCommand(selected);
    return currentAutoCommand;
  }

  public static Command getCurrentAutoCommand() {
    return currentAutoCommand;
  }

  public static String getCurrentAutoId() {
    return currentAutoId;
  }

  public static Set<String> getLoadedAutoIds() {
    reloadAutos(false);
    return Collections.unmodifiableSet(new HashSet<>(autoDefinitions.keySet()));
  }

  private static Command buildAutoCommand(AutoDefinition auto) {
    List<Command> commands = new ArrayList<>();
    BuildContext context = new BuildContext(auto.id());
    for (Object rawBlock : auto.blocks()) {
      Command command = buildBlock(rawBlock, context);
      if (command != null) {
        commands.add(command);
      }
    }

    if (commands.isEmpty()) {
      return Commands.none();
    }

    Command sequence = sequence(commands);
    Path firstPath = context.firstPath();
    if (firstPath == null) {
      return sequence;
    }

    return Commands.sequence(
        Commands.runOnce(() -> orientModules(firstPath.getInitialModuleDirection(poseSupplier))),
        sequence);
  }

  private static Command buildBlock(Object rawBlock, BuildContext context) {
    if (rawBlock instanceof String pathName) {
      return buildPathCommand(pathName, true, context);
    }
    if (!(rawBlock instanceof Map<?, ?> rawMap)) {
      DriverStation.reportWarning("Voyager ignored unsupported block: " + rawBlock, false);
      return Commands.none();
    }

    Map<String, Object> block = typedMap(rawMap);
    String type =
        firstString(block, "type", "kind", "block_type", "blockType")
            .orElse("path")
            .toLowerCase(Locale.ROOT);

    return switch (type) {
      case "path" -> buildPathCommand(
          firstString(block, "name", "path", "path_name", "pathName", "pathKey")
              .orElseThrow(
                  () ->
                      new IllegalArgumentException(
                          "Voyager path block is missing name/path: " + block)),
          true,
          context);
      case "if", "condition", "conditional" -> buildIfBlock(block, context);
      case "loop", "repeat", "repeat_until", "repeatuntil" -> buildLoopBlock(block, context);
      case "interrupt", "interruptable", "interruptible" -> buildInterruptBlock(block, context);
      case "event", "command" -> buildEventBlock(block);
      case "wait", "delay" -> buildWaitBlock(block);
      default -> {
        DriverStation.reportWarning(
            "Voyager ignored unknown block type '" + type + "': " + block, false);
        yield Commands.none();
      }
    };
  }

  private static Command buildPathCommand(
      String pathName, boolean resetPoseIfFirstPath, BuildContext context) {
    Path path = loadPath(pathName);
    if (context.firstPath() == null) {
      context.setFirstPath(path);
    }

    FollowPath.Builder builder =
        new FollowPath.Builder(
                driveSubsystem,
                poseSupplier,
                chassisSpeedsSupplier,
                chassisSpeedsConsumer,
                translationControllerFactory.get(),
                rotationControllerFactory.get(),
                crossTrackControllerFactory.get())
            .withShouldFlip(VoyagerLib::shouldFlipPath);

    if (resetPoseIfFirstPath && !context.hasBuiltPath()) {
      builder.withPoseReset(resetPoseConsumer);
    } else {
      builder.withPoseReset(pose -> {});
    }

    context.markBuiltPath();
    return builder.build(path);
  }

  private static Command buildIfBlock(Map<String, Object> block, BuildContext context) {
    String key = blockKey(block);
    BooleanSupplier condition = conditionFor(key);
    List<Object> onTrue =
        firstList(
                block,
                "on_true",
                "onTrue",
                "true_sequence",
                "trueSequence",
                "true",
                "then",
                "if_true",
                "ifTrue")
            .orElse(List.of());
    List<Object> onFalse =
        firstList(
                block,
                "on_false",
                "onFalse",
                "false_sequence",
                "falseSequence",
                "false",
                "else",
                "if_false",
                "ifFalse")
            .orElse(List.of());
    return BLineCommands.either(
        buildSequence(onTrue, context), buildSequence(onFalse, context), condition);
  }

  private static Command buildLoopBlock(Map<String, Object> block, BuildContext context) {
    String key = blockKey(block);
    List<Object> body =
        firstList(block, "body", "sequence", "path_sequence", "pathSequence", "blocks")
            .orElse(List.of());
    int repetitions = LOOP_REPETITIONS.getOrDefault(key, -1);
    if (repetitions >= 0) {
      List<Command> repeated = new ArrayList<>();
      for (int i = 0; i < repetitions; i++) {
        repeated.add(buildSequence(body, context));
      }
      return sequence(repeated);
    }
    return buildSequence(body, context).repeatedly().until(conditionFor(key));
  }

  private static Command buildInterruptBlock(Map<String, Object> block, BuildContext context) {
    String key = blockKey(block);
    List<Object> body =
        firstList(block, "body", "sequence", "path_sequence", "pathSequence", "blocks")
            .orElse(List.of());
    return buildSequence(body, context).until(conditionFor(key));
  }

  private static Command buildEventBlock(Map<String, Object> block) {
    String key = blockKey(block);
    return eventCommand(key);
  }

  private static Command buildWaitBlock(Map<String, Object> block) {
    double seconds =
        firstDouble(block, "seconds", "time", "duration", "duration_seconds", "durationSeconds")
            .orElse(0.0);
    return Commands.waitSeconds(Math.max(0.0, seconds));
  }

  private static Command buildSequence(List<Object> blocks, BuildContext context) {
    List<Command> commands = new ArrayList<>();
    for (Object block : blocks) {
      commands.add(buildBlock(block, context));
    }
    return sequence(commands);
  }

  private static Command eventCommand(String key) {
    String normalized = normalizeKey(key, "event key");
    if (COMMAND_EVENTS.containsKey(normalized)) {
      return BLineCommands.deferredProxy(() -> COMMAND_EVENTS.get(normalized));
    }
    if (RUNNABLE_EVENTS.containsKey(normalized)) {
      return Commands.runOnce(RUNNABLE_EVENTS.get(normalized));
    }
    warnMissingOnce(
        WARNED_MISSING_EVENTS,
        "Voyager event '" + normalized + "' has no registered command/runnable");
    return Commands.none();
  }

  private static Command sequence(List<Command> commands) {
    List<Command> filtered = new ArrayList<>();
    for (Command command : commands) {
      if (command != null) {
        filtered.add(command);
      }
    }
    if (filtered.isEmpty()) {
      return Commands.none();
    }
    return BLineCommands.sequence(filtered.toArray(Command[]::new));
  }

  private static BooleanSupplier conditionFor(String key) {
    String normalized = normalizeKey(key, "condition key");
    BooleanSupplier condition = CONDITIONS.get(normalized);
    if (condition != null) {
      return condition;
    }
    warnMissingOnce(
        WARNED_MISSING_CONDITIONS,
        "Voyager condition '" + normalized + "' is not registered; using false");
    return () -> false;
  }

  private static Path loadPath(String pathName) {
    java.nio.file.Path file = resolvePathFile(pathName);
    Map<String, Object> json = readJsonObject(file);
    return parseVoyagerPath(json, pathName);
  }

  private static Path parseVoyagerPath(Map<String, Object> json, String pathName) {
    List<Object> rawWaypoints =
        firstList(json, "waypoints", "points", "translation_targets", "translationTargets")
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        "Voyager path '" + pathName + "' has no waypoints array"));
    if (rawWaypoints.isEmpty()) {
      throw new IllegalArgumentException(
          "Voyager path '" + pathName + "' has an empty waypoints array");
    }

    List<WaypointSpec> waypoints = new ArrayList<>();
    for (Object rawWaypoint : rawWaypoints) {
      if (!(rawWaypoint instanceof Map<?, ?> rawMap)) {
        continue;
      }
      waypoints.add(parseWaypoint(typedMap(rawMap)));
    }
    if (waypoints.isEmpty()) {
      throw new IllegalArgumentException("Voyager path '" + pathName + "' has no valid waypoints");
    }

    List<PositionedRotationTarget> rotations = parsePositionedRotations(json, waypoints);
    List<PositionedEventTrigger> events = parsePositionedEvents(json);
    Path.PathConstraints constraints =
        parseConstraints(json, waypoints.size(), waypoints.size() + rotations.size());
    List<Path.PathElement> elements = buildPathElements(waypoints, rotations, events);
    return new Path(elements, constraints, defaultGlobalConstraints);
  }

  private static WaypointSpec parseWaypoint(Map<String, Object> waypointJson) {
    Map<String, Object> translation =
        firstMap(waypointJson, "translation", "translationTarget", "translation_target")
            .orElse(waypointJson);
    double x = requiredDouble(translation, "waypoint x", "x", "x_meters", "xMeters");
    double y = requiredDouble(translation, "waypoint y", "y", "y_meters", "yMeters");

    Map<String, Object> rotationContainer =
        firstMap(waypointJson, "rotation_target", "rotationTarget").orElse(waypointJson);
    Rotation2d rotation = parseRotation(rotationContainer).orElse(Rotation2d.fromDegrees(0.0));
    double handoff =
        firstDouble(
                waypointJson,
                "handoff",
                "handoff_radius",
                "handoffRadius",
                "intermediate_handoff_radius_meters",
                "intermediateHandoffRadiusMeters")
            .orElse(Double.NaN);
    boolean profiled =
        firstBoolean(waypointJson, "profiled", "profiled_rotation", "profiledRotation")
            .orElse(true);
    return new WaypointSpec(new Translation2d(x, y), rotation, handoff, profiled);
  }

  private static List<PositionedRotationTarget> parsePositionedRotations(
      Map<String, Object> json, List<WaypointSpec> waypoints) {
    List<PositionedRotationTarget> targets = new ArrayList<>();
    for (Object raw :
        firstList(json, "rotation_targets", "rotationTargets", "rotations", "rotationTargetsJson")
            .orElse(List.of())) {
      if (!(raw instanceof Map<?, ?> rawMap)) {
        continue;
      }
      Map<String, Object> rotationJson = typedMap(rawMap);
      Rotation2d rotation = parseRotation(rotationJson).orElse(null);
      if (rotation == null) {
        DriverStation.reportWarning(
            "Voyager ignored rotation target with no angle: " + rotationJson, false);
        continue;
      }
      double position = pathPosition(rotationJson, 0.5);
      boolean profiled =
          firstBoolean(rotationJson, "profiled", "profiled_rotation", "profiledRotation")
              .orElse(true);
      Optional<Integer> endpoint = endpointWaypointIndex(position, waypoints.size());
      if (endpoint.isPresent()) {
        int index = endpoint.get();
        WaypointSpec current = waypoints.get(index);
        waypoints.set(index, current.withRotation(rotation, profiled));
      } else {
        SegmentRatio segmentRatio = segmentRatio(position, waypoints.size());
        targets.add(
            new PositionedRotationTarget(
                segmentRatio.segment(), segmentRatio.tRatio(), rotation, profiled, position));
      }
    }
    targets.sort(PositionedRotationTarget::compareTo);
    return targets;
  }

  private static List<PositionedEventTrigger> parsePositionedEvents(Map<String, Object> json) {
    List<PositionedEventTrigger> triggers = new ArrayList<>();
    for (Object raw :
        firstList(
                json, "events", "event_triggers", "eventTriggers", "event_markers", "eventMarkers")
            .orElse(List.of())) {
      if (!(raw instanceof Map<?, ?> rawMap)) {
        continue;
      }
      Map<String, Object> eventJson = typedMap(rawMap);
      Optional<String> key =
          firstString(eventJson, "name", "key", "event", "eventName", "lib_key", "libKey");
      if (key.isEmpty()) {
        DriverStation.reportWarning(
            "Voyager ignored event trigger with no name/key: " + eventJson, false);
        continue;
      }
      double position = pathPosition(eventJson, 0.5);
      triggers.add(new PositionedEventTrigger(position, key.get()));
    }
    triggers.sort(PositionedEventTrigger::compareTo);
    return triggers;
  }

  private static List<Path.PathElement> buildPathElements(
      List<WaypointSpec> waypoints,
      List<PositionedRotationTarget> rotations,
      List<PositionedEventTrigger> events) {
    Map<Integer, List<PositionedRotationTarget>> rotationsBySegment = new HashMap<>();
    for (PositionedRotationTarget target : rotations) {
      rotationsBySegment.computeIfAbsent(target.segment(), unused -> new ArrayList<>()).add(target);
    }

    Map<Integer, List<PositionedEventTrigger>> eventsBySegment = new HashMap<>();
    for (PositionedEventTrigger event : events) {
      SegmentRatio sr = segmentRatio(event.position(), waypoints.size());
      eventsBySegment
          .computeIfAbsent(sr.segment(), unused -> new ArrayList<>())
          .add(new PositionedEventTrigger(sr.segment() + sr.tRatio(), event.key()));
    }

    List<Path.PathElement> elements = new ArrayList<>();
    for (int i = 0; i < waypoints.size(); i++) {
      elements.add(waypoints.get(i).toWaypoint());
      if (i >= waypoints.size() - 1) {
        continue;
      }

      List<PositionedRotationTarget> segmentRotations =
          rotationsBySegment.getOrDefault(i, List.of());
      List<PositionedEventTrigger> segmentEvents = eventsBySegment.getOrDefault(i, List.of());
      List<IntermediateElement> intermediates = new ArrayList<>();
      for (PositionedRotationTarget target : segmentRotations) {
        intermediates.add(
            new IntermediateElement(
                target.tRatio(),
                new Path.RotationTarget(target.rotation(), target.tRatio(), target.profiled())));
      }
      for (PositionedEventTrigger event : segmentEvents) {
        double tRatio = clamp(event.position() - Math.floor(event.position()), 0.0, 1.0);
        intermediates.add(
            new IntermediateElement(tRatio, new Path.EventTrigger(tRatio, event.key())));
      }
      intermediates.sort(IntermediateElement::compareTo);
      for (IntermediateElement intermediate : intermediates) {
        elements.add(intermediate.element());
      }
    }
    return elements;
  }

  private static Path.PathConstraints parseConstraints(
      Map<String, Object> json, int translationOrdinalCount, int rotationOrdinalCount) {
    Path.PathConstraints constraints = new Path.PathConstraints();
    Map<String, Object> constraintsMap =
        firstMap(json, "constraints", "kinematic_constraints", "kinematicConstraints").orElse(json);

    applyGlobalConstraint(
        constraintsMap,
        constraints::setMaxVelocityMetersPerSec,
        "max_translational_velocity",
        "maxVelocityMetersPerSec",
        "max_velocity_meters_per_sec");
    applyGlobalConstraint(
        constraintsMap,
        constraints::setMaxAccelerationMetersPerSec2,
        "max_translational_acceleration",
        "maxAccelerationMetersPerSec2",
        "max_acceleration_meters_per_sec2");
    applyGlobalConstraint(
        constraintsMap,
        constraints::setMaxVelocityDegPerSec,
        "max_rotational_velocity",
        "maxVelocityDegPerSec",
        "max_velocity_deg_per_sec");
    applyGlobalConstraint(
        constraintsMap,
        constraints::setMaxAccelerationDegPerSec2,
        "max_rotational_acceleration",
        "maxAccelerationDegPerSec2",
        "max_acceleration_deg_per_sec2");
    applyGlobalConstraint(
        constraintsMap,
        constraints::setMinVelocityMetersPerSec,
        "min_translational_velocity",
        "minVelocityMetersPerSec",
        "min_velocity_meters_per_sec");
    applyGlobalConstraint(
        constraintsMap,
        constraints::setMinVelocityDegPerSec,
        "min_rotational_velocity",
        "minVelocityDegPerSec",
        "min_velocity_deg_per_sec");

    firstDouble(
            constraintsMap,
            "end_translation_tolerance_meters",
            "endTranslationToleranceMeters",
            "translation_tolerance",
            "translationTolerance")
        .ifPresent(constraints::setEndTranslationToleranceMeters);
    firstDouble(
            constraintsMap,
            "end_rotation_tolerance_deg",
            "endRotationToleranceDeg",
            "rotation_tolerance_deg",
            "rotationToleranceDeg")
        .ifPresent(constraints::setEndRotationToleranceDeg);

    List<Object> zones =
        firstList(json, "constraint_zones", "constraintZones", "zones")
            .or(() -> firstList(constraintsMap, "constraint_zones", "constraintZones", "zones"))
            .orElse(List.of());

    applyRangedConstraints(
        constraints,
        zones,
        translationOrdinalCount,
        false,
        RangedSetter.MAX_TRANSLATION_VELOCITY,
        "max_translational_velocity",
        "maxVelocityMetersPerSec",
        "max_velocity_meters_per_sec");
    applyRangedConstraints(
        constraints,
        zones,
        translationOrdinalCount,
        false,
        RangedSetter.MAX_TRANSLATION_ACCELERATION,
        "max_translational_acceleration",
        "maxAccelerationMetersPerSec2",
        "max_acceleration_meters_per_sec2");
    applyRangedConstraints(
        constraints,
        zones,
        translationOrdinalCount,
        false,
        RangedSetter.MIN_TRANSLATION_VELOCITY,
        "min_translational_velocity",
        "minVelocityMetersPerSec",
        "min_velocity_meters_per_sec");
    applyRangedConstraints(
        constraints,
        zones,
        rotationOrdinalCount,
        true,
        RangedSetter.MAX_ROTATION_VELOCITY,
        "max_rotational_velocity",
        "maxVelocityDegPerSec",
        "max_velocity_deg_per_sec");
    applyRangedConstraints(
        constraints,
        zones,
        rotationOrdinalCount,
        true,
        RangedSetter.MAX_ROTATION_ACCELERATION,
        "max_rotational_acceleration",
        "maxAccelerationDegPerSec2",
        "max_acceleration_deg_per_sec2");
    applyRangedConstraints(
        constraints,
        zones,
        rotationOrdinalCount,
        true,
        RangedSetter.MIN_ROTATION_VELOCITY,
        "min_rotational_velocity",
        "minVelocityDegPerSec",
        "min_velocity_deg_per_sec");

    parseBLineRangedArray(
            constraintsMap,
            translationOrdinalCount,
            "max_velocity_meters_per_sec",
            "maxVelocityMetersPerSec")
        .ifPresent(rc -> constraints.setMaxVelocityMetersPerSec(rc));
    parseBLineRangedArray(
            constraintsMap,
            translationOrdinalCount,
            "max_acceleration_meters_per_sec2",
            "maxAccelerationMetersPerSec2")
        .ifPresent(rc -> constraints.setMaxAccelerationMetersPerSec2(rc));
    parseBLineRangedArray(
            constraintsMap,
            translationOrdinalCount,
            "min_velocity_meters_per_sec",
            "minVelocityMetersPerSec")
        .ifPresent(rc -> constraints.setMinVelocityMetersPerSec(rc));
    parseBLineRangedArray(
            constraintsMap,
            rotationOrdinalCount,
            "max_velocity_deg_per_sec",
            "maxVelocityDegPerSec")
        .ifPresent(rc -> constraints.setMaxVelocityDegPerSec(rc));
    parseBLineRangedArray(
            constraintsMap,
            rotationOrdinalCount,
            "max_acceleration_deg_per_sec2",
            "maxAccelerationDegPerSec2")
        .ifPresent(rc -> constraints.setMaxAccelerationDegPerSec2(rc));
    parseBLineRangedArray(
            constraintsMap,
            rotationOrdinalCount,
            "min_velocity_deg_per_sec",
            "minVelocityDegPerSec")
        .ifPresent(rc -> constraints.setMinVelocityDegPerSec(rc));

    return constraints;
  }

  private static Optional<Path.RangedConstraint[]> parseBLineRangedArray(
      Map<String, Object> object, int ordinalCount, String... keys) {
    Optional<List<Object>> array = firstList(object, keys);
    if (array.isEmpty()) {
      return Optional.empty();
    }
    List<Path.RangedConstraint> ranges = new ArrayList<>();
    for (Object raw : array.get()) {
      if (!(raw instanceof Map<?, ?> rawMap)) {
        continue;
      }
      Map<String, Object> map = typedMap(rawMap);
      OptionalDouble value = firstDouble(map, "value");
      if (value.isEmpty()) {
        continue;
      }
      int start =
          roundedOrdinal(
              firstDouble(
                      map,
                      "start_ordinal",
                      "startOrdinal",
                      "start",
                      "start_position",
                      "startPosition")
                  .orElse(0.0),
              ordinalCount);
      int end =
          roundedOrdinal(
              firstDouble(map, "end_ordinal", "endOrdinal", "end", "end_position", "endPosition")
                  .orElse(ordinalCount - 1.0),
              ordinalCount);
      ranges.add(
          new Path.RangedConstraint(
              value.getAsDouble(), Math.min(start, end), Math.max(start, end)));
    }
    return ranges.isEmpty()
        ? Optional.empty()
        : Optional.of(ranges.toArray(Path.RangedConstraint[]::new));
  }

  private static void applyRangedConstraints(
      Path.PathConstraints constraints,
      List<Object> zones,
      int ordinalCount,
      boolean rotational,
      RangedSetter setter,
      String... valueKeys) {
    List<Path.RangedConstraint> ranges = new ArrayList<>();
    for (Object raw : zones) {
      if (!(raw instanceof Map<?, ?> rawMap)) {
        continue;
      }
      Map<String, Object> zone = typedMap(rawMap);
      OptionalDouble value = firstDouble(zone, valueKeys);
      if (value.isEmpty()) {
        Map<String, Object> nested = firstMap(zone, "constraints", "constraint").orElse(Map.of());
        value = firstDouble(nested, valueKeys);
      }
      if (value.isEmpty()) {
        continue;
      }

      double startRaw =
          firstDouble(
                  zone,
                  rotational ? "rotation_start_position" : "translation_start_position",
                  rotational ? "rotationStartPosition" : "translationStartPosition",
                  rotational ? "rotationStart" : "translationStart",
                  "start_position",
                  "startPosition",
                  "start")
              .orElse(0.0);
      double endRaw =
          firstDouble(
                  zone,
                  rotational ? "rotation_end_position" : "translation_end_position",
                  rotational ? "rotationEndPosition" : "translationEndPosition",
                  rotational ? "rotationEnd" : "translationEnd",
                  "end_position",
                  "endPosition",
                  "end")
              .orElse(ordinalCount - 1.0);
      int start = roundedOrdinal(startRaw, ordinalCount);
      int end = roundedOrdinal(endRaw, ordinalCount);
      ranges.add(
          new Path.RangedConstraint(
              value.getAsDouble(), Math.min(start, end), Math.max(start, end)));
    }
    if (!ranges.isEmpty()) {
      Path.RangedConstraint[] array = ranges.toArray(Path.RangedConstraint[]::new);
      switch (setter) {
        case MAX_TRANSLATION_VELOCITY -> constraints.setMaxVelocityMetersPerSec(array);
        case MAX_TRANSLATION_ACCELERATION -> constraints.setMaxAccelerationMetersPerSec2(array);
        case MIN_TRANSLATION_VELOCITY -> constraints.setMinVelocityMetersPerSec(array);
        case MAX_ROTATION_VELOCITY -> constraints.setMaxVelocityDegPerSec(array);
        case MAX_ROTATION_ACCELERATION -> constraints.setMaxAccelerationDegPerSec2(array);
        case MIN_ROTATION_VELOCITY -> constraints.setMinVelocityDegPerSec(array);
      }
    }
  }

  private static void applyGlobalConstraint(
      Map<String, Object> object, Consumer<Double> setter, String... keys) {
    OptionalDouble value = firstDouble(object, keys);
    if (value.isPresent()) {
      setter.accept(value.getAsDouble());
    }
  }

  private static SegmentRatio segmentRatio(double position, int waypointCount) {
    if (waypointCount < 2) {
      return new SegmentRatio(0, 0.0);
    }
    double clamped = clamp(position, 0.0, waypointCount - 1.0);
    int segment = (int) Math.floor(clamped);
    if (segment >= waypointCount - 1) {
      segment = waypointCount - 2;
      return new SegmentRatio(segment, 1.0);
    }
    return new SegmentRatio(segment, clamp(clamped - segment, 0.0, 1.0));
  }

  private static Optional<Integer> endpointWaypointIndex(double position, int waypointCount) {
    if (waypointCount == 0) {
      return Optional.empty();
    }
    double rounded = Math.rint(position);
    if (Math.abs(position - rounded) > 1e-9) {
      return Optional.empty();
    }
    int index = (int) rounded;
    if (index <= 0) {
      return Optional.of(0);
    }
    if (index >= waypointCount - 1) {
      return Optional.of(waypointCount - 1);
    }
    return Optional.of(index);
  }

  private static int roundedOrdinal(double raw, int count) {
    if (count <= 0) {
      return 0;
    }
    return (int) clamp(Math.round(raw), 0, count - 1);
  }

  private static double pathPosition(Map<String, Object> object, double fallback) {
    return firstDouble(
            object,
            "position",
            "path_position",
            "pathPosition",
            "ordinal",
            "waypointOrdinal",
            "waypoint_ordinal")
        .orElse(fallback);
  }

  private static Optional<Rotation2d> parseRotation(Map<String, Object> object) {
    OptionalDouble radians =
        firstDouble(
            object,
            "rotation_radians",
            "rotationRadians",
            "heading_radians",
            "headingRadians",
            "angle_radians",
            "angleRadians",
            "theta_radians",
            "thetaRadians",
            "radians",
            "rad");
    if (radians.isPresent()) {
      return Optional.of(Rotation2d.fromRadians(radians.getAsDouble()));
    }
    OptionalDouble degrees =
        firstDouble(
            object,
            "angle",
            "rotation",
            "heading",
            "theta",
            "targetRotation",
            "target_rotation",
            "targetHeading",
            "target_heading",
            "targetAngle",
            "target_angle",
            "rotationDeg",
            "rotation_deg",
            "rotationDegrees",
            "rotation_degrees",
            "headingDeg",
            "heading_deg",
            "headingDegrees",
            "heading_degrees",
            "degrees",
            "deg",
            "r");
    return degrees.isPresent()
        ? Optional.of(Rotation2d.fromDegrees(degrees.getAsDouble()))
        : Optional.empty();
  }

  private static void reloadAutos(boolean throwIfMissing) {
    synchronized (AUTO_LOCK) {
      java.nio.file.Path configPath = voyagerDirectory.resolve("auto_config.json");
      if (!java.nio.file.Files.isRegularFile(configPath)) {
        String message = "Voyager auto_config.json not found: " + configPath.toAbsolutePath();
        if (throwIfMissing) {
          throw new IllegalStateException(message);
        }
        DriverStation.reportWarning(message, false);
        return;
      }

      try {
        long modified = java.nio.file.Files.getLastModifiedTime(configPath).toMillis();
        if (modified == autoConfigLastModifiedMillis && !autoDefinitions.isEmpty()) {
          return;
        }
        Map<String, Object> root = readJsonObject(configPath);
        autoDefinitions = parseAutoDefinitions(root);
        autoConfigLastModifiedMillis = modified;
      } catch (IOException ex) {
        throw new IllegalStateException(
            "Failed to read Voyager auto_config.json: " + configPath, ex);
      }
    }
  }

  private static Map<String, AutoDefinition> parseAutoDefinitions(Map<String, Object> root) {
    List<Object> autos = firstList(root, "autos", "auto", "autonomous").orElse(List.of(root));
    Map<String, AutoDefinition> parsed = new LinkedHashMap<>();
    for (Object rawAuto : autos) {
      if (!(rawAuto instanceof Map<?, ?> rawMap)) {
        continue;
      }
      Map<String, Object> auto = typedMap(rawMap);
      String id =
          firstString(auto, "id", "name", "auto_id", "autoId").orElse("auto" + parsed.size());
      List<Object> blocks =
          firstList(auto, "path_sequence", "pathSequence", "sequence", "blocks", "commands")
              .orElse(List.of());
      parsed.put(id, new AutoDefinition(id, blocks));
    }
    if (parsed.isEmpty()) {
      throw new IllegalArgumentException("Voyager auto_config.json did not contain any autos");
    }
    return Map.copyOf(parsed);
  }

  private static String firstAutoId() {
    return autoDefinitions.keySet().stream().findFirst().orElse("");
  }

  private static java.nio.file.Path resolvePathFile(String pathName) {
    String fileName = stripJsonExtension(pathName) + ".json";
    List<java.nio.file.Path> candidates =
        List.of(
            voyagerDirectory.resolve(fileName),
            defaultVoyagerDirectory().resolve(fileName),
            voyagerDirectory.resolve("paths").resolve(fileName),
            defaultVoyagerDirectory().resolve("paths").resolve(fileName));
    for (java.nio.file.Path candidate : candidates) {
      if (java.nio.file.Files.isRegularFile(candidate)) {
        return candidate;
      }
    }
    throw new IllegalStateException(
        "Voyager path JSON not found for '" + pathName + "'. Checked: " + candidates);
  }

  private static String stripJsonExtension(String name) {
    String trimmed = normalizeKey(name, "path name");
    return trimmed.endsWith(".json") ? trimmed.substring(0, trimmed.length() - 5) : trimmed;
  }

  private static boolean shouldFlipPath() {
    return flipIfRed
        && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  private static void orientModules(Rotation2d direction) {
    if (direction == null) {
      return;
    }
    if (moduleOrientationConsumer != null) {
      moduleOrientationConsumer.accept(direction);
      return;
    }
    if (reflectedModuleOrientationMethod == null || driveSubsystem == null) {
      return;
    }
    try {
      reflectedModuleOrientationMethod.invoke(driveSubsystem, direction);
    } catch (IllegalAccessException | InvocationTargetException ex) {
      DriverStation.reportWarning(
          "Voyager failed to pre-orient modules: " + ex.getMessage(), false);
    }
  }

  private static Method findModuleOrientationMethod(Subsystem drive) {
    for (String name : List.of("setModuleOrientations", "setModuleOrientation", "orientModules")) {
      try {
        Method method = drive.getClass().getMethod(name, Rotation2d.class);
        method.setAccessible(true);
        return method;
      } catch (NoSuchMethodException ignored) {
        // Try next common name.
      }
    }
    return null;
  }

  private static PIDController copyPidController(PIDController source, boolean continuous) {
    PIDController copy = new PIDController(source.getP(), source.getI(), source.getD());
    if (continuous) {
      copy.enableContinuousInput(-Math.PI, Math.PI);
    }
    return copy;
  }

  private static void ensureConfigured() {
    if (driveSubsystem == null
        || poseSupplier == null
        || resetPoseConsumer == null
        || chassisSpeedsSupplier == null
        || chassisSpeedsConsumer == null) {
      throw new IllegalStateException(
          "VoyagerLib.configure(...) must be called before running autos");
    }
  }

  private static java.nio.file.Path defaultVoyagerDirectory() {
    return Filesystem.getDeployDirectory().toPath().resolve("voyager");
  }

  private static Map<String, Object> readJsonObject(java.nio.file.Path file) {
    try {
      Object root = Json.parse(java.nio.file.Files.readString(file, StandardCharsets.UTF_8));
      if (!(root instanceof Map<?, ?> rawMap)) {
        throw new IllegalArgumentException(
            "Expected top-level JSON object in " + file.toAbsolutePath());
      }
      return typedMap(rawMap);
    } catch (IOException ex) {
      throw new IllegalStateException("Failed to read JSON file: " + file.toAbsolutePath(), ex);
    }
  }

  private static Map<String, Object> typedMap(Map<?, ?> raw) {
    Map<String, Object> map = new LinkedHashMap<>();
    for (Map.Entry<?, ?> entry : raw.entrySet()) {
      if (entry.getKey() != null) {
        map.put(String.valueOf(entry.getKey()), entry.getValue());
      }
    }
    return map;
  }

  private static Optional<Map<String, Object>> firstMap(
      Map<String, Object> object, String... keys) {
    for (String key : keys) {
      Object raw = object.get(key);
      if (raw instanceof Map<?, ?> rawMap) {
        return Optional.of(typedMap(rawMap));
      }
    }
    return Optional.empty();
  }

  @SuppressWarnings("unchecked")
  private static Optional<List<Object>> firstList(Map<String, Object> object, String... keys) {
    for (String key : keys) {
      Object raw = object.get(key);
      if (raw instanceof List<?> rawList) {
        return Optional.of((List<Object>) rawList);
      }
    }
    return Optional.empty();
  }

  private static Optional<String> firstString(Map<String, Object> object, String... keys) {
    for (String key : keys) {
      Object raw = object.get(key);
      if (raw instanceof String text && !text.trim().isEmpty()) {
        return Optional.of(text.trim());
      }
      if (raw instanceof Number || raw instanceof Boolean) {
        return Optional.of(String.valueOf(raw));
      }
    }
    return Optional.empty();
  }

  private static OptionalBoolean firstBoolean(Map<String, Object> object, String... keys) {
    for (String key : keys) {
      Object raw = object.get(key);
      if (raw instanceof Boolean bool) {
        return OptionalBoolean.of(bool);
      }
      if (raw instanceof String text) {
        String cleaned = text.trim().toLowerCase(Locale.ROOT);
        if (cleaned.equals("true") || cleaned.equals("yes") || cleaned.equals("1")) {
          return OptionalBoolean.of(true);
        }
        if (cleaned.equals("false") || cleaned.equals("no") || cleaned.equals("0")) {
          return OptionalBoolean.of(false);
        }
      }
    }
    return OptionalBoolean.empty();
  }

  private static OptionalDouble firstDouble(Map<String, Object> object, String... keys) {
    for (String key : keys) {
      Object raw = object.get(key);
      OptionalDouble parsed = toDouble(raw);
      if (parsed.isPresent()) {
        return parsed;
      }
    }
    return OptionalDouble.empty();
  }

  private static OptionalDouble toDouble(Object raw) {
    if (raw instanceof Number number) {
      return OptionalDouble.of(number.doubleValue());
    }
    if (raw instanceof String text) {
      try {
        return OptionalDouble.of(Double.parseDouble(text.trim()));
      } catch (NumberFormatException ignored) {
        return OptionalDouble.empty();
      }
    }
    return OptionalDouble.empty();
  }

  private static double requiredDouble(Map<String, Object> object, String label, String... keys) {
    return firstDouble(object, keys)
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    "Missing numeric " + label + " from keys " + Arrays.toString(keys)));
  }

  private static String blockKey(Map<String, Object> block) {
    return firstString(
            block,
            "condition",
            "condition_key",
            "conditionKey",
            "name",
            "id",
            "key",
            "event",
            "eventName")
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    "Voyager block needs a name/id/key/condition: " + block));
  }

  private static String normalizeKey(String key, String label) {
    if (key == null || key.trim().isEmpty()) {
      throw new IllegalArgumentException(label + " cannot be blank");
    }
    return key.trim();
  }

  private static void warnMissingOnce(Set<String> cache, String message) {
    if (cache.add(message)) {
      DriverStation.reportWarning(message, false);
    }
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private enum RangedSetter {
    MAX_TRANSLATION_VELOCITY,
    MAX_TRANSLATION_ACCELERATION,
    MIN_TRANSLATION_VELOCITY,
    MAX_ROTATION_VELOCITY,
    MAX_ROTATION_ACCELERATION,
    MIN_ROTATION_VELOCITY
  }

  private record AutoDefinition(String id, List<Object> blocks) {}

  private static final class BuildContext {
    private final String autoId;
    private Path firstPath;
    private boolean builtAnyPath;

    BuildContext(String autoId) {
      this.autoId = autoId;
    }

    Path firstPath() {
      return firstPath;
    }

    void setFirstPath(Path path) {
      if (firstPath == null) {
        firstPath = path;
      }
    }

    boolean hasBuiltPath() {
      return builtAnyPath;
    }

    void markBuiltPath() {
      builtAnyPath = true;
    }

    @Override
    public String toString() {
      return autoId;
    }
  }

  private record WaypointSpec(
      Translation2d translation,
      Rotation2d rotation,
      double handoffRadiusMeters,
      boolean profiledRotation) {
    Path.Waypoint toWaypoint() {
      if (Double.isFinite(handoffRadiusMeters)) {
        return new Path.Waypoint(translation, handoffRadiusMeters, rotation, profiledRotation);
      }
      return new Path.Waypoint(translation, rotation, profiledRotation);
    }

    WaypointSpec withRotation(Rotation2d newRotation, boolean profiled) {
      return new WaypointSpec(translation, newRotation, handoffRadiusMeters, profiled);
    }
  }

  private record SegmentRatio(int segment, double tRatio) {}

  private record PositionedRotationTarget(
      int segment, double tRatio, Rotation2d rotation, boolean profiled, double originalPosition)
      implements Comparable<PositionedRotationTarget> {
    @Override
    public int compareTo(PositionedRotationTarget other) {
      int bySegment = Integer.compare(segment, other.segment);
      return bySegment != 0 ? bySegment : Double.compare(tRatio, other.tRatio);
    }
  }

  private record PositionedEventTrigger(double position, String key)
      implements Comparable<PositionedEventTrigger> {
    @Override
    public int compareTo(PositionedEventTrigger other) {
      return Double.compare(position, other.position);
    }
  }

  private record IntermediateElement(double tRatio, Path.PathElement element)
      implements Comparable<IntermediateElement> {
    @Override
    public int compareTo(IntermediateElement other) {
      return Double.compare(tRatio, other.tRatio);
    }
  }

  private record OptionalBoolean(boolean present, boolean value) {
    static OptionalBoolean of(boolean value) {
      return new OptionalBoolean(true, value);
    }

    static OptionalBoolean empty() {
      return new OptionalBoolean(false, false);
    }

    boolean orElse(boolean fallback) {
      return present ? value : fallback;
    }
  }

  private static final class Json {
    private final String source;
    private int index;

    private Json(String source) {
      this.source = source;
    }

    static Object parse(String source) {
      Json parser = new Json(source);
      Object value = parser.parseValue();
      parser.skipWhitespace();
      if (!parser.isAtEnd()) {
        throw parser.error("Unexpected trailing content");
      }
      return value;
    }

    private Object parseValue() {
      skipWhitespace();
      if (isAtEnd()) {
        throw error("Unexpected end of JSON");
      }
      char current = peek();
      return switch (current) {
        case '{' -> parseObject();
        case '[' -> parseArray();
        case '"' -> parseString();
        case 't' -> parseLiteral("true", Boolean.TRUE);
        case 'f' -> parseLiteral("false", Boolean.FALSE);
        case 'n' -> parseLiteral("null", null);
        default -> {
          if (current == '-' || Character.isDigit(current)) {
            yield parseNumber();
          }
          throw error("Unexpected character '" + current + "'");
        }
      };
    }

    private Map<String, Object> parseObject() {
      expect('{');
      Map<String, Object> object = new LinkedHashMap<>();
      skipWhitespace();
      if (consumeIf('}')) {
        return object;
      }
      while (true) {
        skipWhitespace();
        String key = parseString();
        skipWhitespace();
        expect(':');
        Object value = parseValue();
        object.put(key, value);
        skipWhitespace();
        if (consumeIf('}')) {
          return object;
        }
        expect(',');
      }
    }

    private List<Object> parseArray() {
      expect('[');
      List<Object> array = new ArrayList<>();
      skipWhitespace();
      if (consumeIf(']')) {
        return array;
      }
      while (true) {
        array.add(parseValue());
        skipWhitespace();
        if (consumeIf(']')) {
          return array;
        }
        expect(',');
      }
    }

    private String parseString() {
      expect('"');
      StringBuilder builder = new StringBuilder();
      while (!isAtEnd()) {
        char current = advance();
        if (current == '"') {
          return builder.toString();
        }
        if (current != '\\') {
          builder.append(current);
          continue;
        }
        if (isAtEnd()) {
          throw error("Unterminated escape sequence");
        }
        char escaped = advance();
        switch (escaped) {
          case '"' -> builder.append('"');
          case '\\' -> builder.append('\\');
          case '/' -> builder.append('/');
          case 'b' -> builder.append('\b');
          case 'f' -> builder.append('\f');
          case 'n' -> builder.append('\n');
          case 'r' -> builder.append('\r');
          case 't' -> builder.append('\t');
          case 'u' -> builder.append(parseUnicodeEscape());
          default -> throw error("Invalid escape sequence \\" + escaped + "'");
        }
      }
      throw error("Unterminated JSON string");
    }

    private char parseUnicodeEscape() {
      if (index + 4 > source.length()) {
        throw error("Incomplete unicode escape");
      }
      String hex = source.substring(index, index + 4);
      index += 4;
      try {
        return (char) Integer.parseInt(hex, 16);
      } catch (NumberFormatException ex) {
        throw error("Invalid unicode escape: " + hex);
      }
    }

    private Object parseLiteral(String literal, Object value) {
      if (!source.startsWith(literal, index)) {
        throw error("Expected literal " + literal);
      }
      index += literal.length();
      return value;
    }

    private Number parseNumber() {
      int start = index;
      consumeIf('-');
      if (consumeIf('0')) {
        // Leading zero is allowed only as the whole integer part.
      } else {
        consumeDigits();
      }
      if (consumeIf('.')) {
        consumeDigits();
      }
      if (consumeIf('e') || consumeIf('E')) {
        consumeIf('+');
        consumeIf('-');
        consumeDigits();
      }
      String number = source.substring(start, index);
      try {
        if (number.contains(".") || number.contains("e") || number.contains("E")) {
          return Double.parseDouble(number);
        }
        long asLong = Long.parseLong(number);
        if (asLong >= Integer.MIN_VALUE && asLong <= Integer.MAX_VALUE) {
          return (int) asLong;
        }
        return asLong;
      } catch (NumberFormatException ex) {
        throw error("Invalid JSON number: " + number);
      }
    }

    private void consumeDigits() {
      int start = index;
      while (!isAtEnd() && Character.isDigit(peek())) {
        index++;
      }
      if (index == start) {
        throw error("Expected digit");
      }
    }

    private void skipWhitespace() {
      while (!isAtEnd()) {
        char c = peek();
        if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
          index++;
        } else {
          return;
        }
      }
    }

    private void expect(char expected) {
      if (isAtEnd() || advance() != expected) {
        throw error("Expected '" + expected + "'");
      }
    }

    private boolean consumeIf(char expected) {
      if (!isAtEnd() && peek() == expected) {
        index++;
        return true;
      }
      return false;
    }

    private char peek() {
      return source.charAt(index);
    }

    private char advance() {
      return source.charAt(index++);
    }

    private boolean isAtEnd() {
      return index >= source.length();
    }

    private IllegalArgumentException error(String message) {
      return new IllegalArgumentException(message + " at JSON index " + index);
    }
  }
}
