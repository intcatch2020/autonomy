package com.platypus.android.server;

import android.util.Log;

import com.platypus.crw.data.UtmPose;

import org.json.JSONObject;

import java.lang.reflect.Array;
import java.lang.reflect.Constructor;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import javolution.io.Struct;
import javolution.lang.Reflection;

/**
 * Created by jason on 8/11/17.
 */

public class VehicleState
{

		private final int NUMBER_OF_SAMPLER_JARS = 4;
		private VehicleServerImpl _serverImpl;
		private static String logTag = "AP";
		private AtomicBoolean[] jar_available = new AtomicBoolean[NUMBER_OF_SAMPLER_JARS];
		private HashMap<String, State> state_map = new HashMap<>();

		enum States
		{
				EXAMPLE_STATE("example_state"),
				EXAMPLE_VALUE("example_value"),
				EXAMPLE_ARRAY("example_array"),
				EC("EC"),
				DO("DO"),
				T("T"),
				PH("PH"),
				WATER_DEPTH("water_depth"),
				CURRENT_POSE("current_pose"), // location, UTM
				HOME_POSE("home_pose"), // home location, UTM
				FIRST_POSE("first_pose"), // first GPS location, UTM
				ELAPSED_TIME("elapsed_time"),
				TIME_SINCE_OPERATOR("time_since_operator"), // time elapsed past last time operator detected
				BATTERY_VOLTAGE("battery_voltage"),
				IS_CONNECTED("is_connected"),
				IS_AUTONOMOUS("is_autonomous"),
				HAS_FIRST_AUTONOMY("has_first_autonomy"),
				HAS_FIRST_GPS("has_first_gps"),
				IS_RUNNING("is_running"),
				IS_GOING_HOME("is_going_home"),
				IS_TAKING_SAMPLE("is_taking_sample"),
				NEXT_AVAILABLE_JAR("next_jar"),
				JARS_AVAILABLE("jars_available"),
				ALWAYS_TRUE("always_true"),
				ALWAYS_FALSE("always_false"),
				RC_OVERRIDE_IS_ON("rc_override");

				final String name;
				States(final String _name) { name = _name; }
		}

		enum LogOption
		{
				NEVER,
				PRINT_ANY_SET,
				PRINT_WHEN_CHANGED
		}

		abstract class State <S, F>
		{
				/*
					Generic for storing and retrieving the state
					Template type F is the value used in get and set, which may be different than S.
					For example, S is AtomicBoolean and F is Boolean for threadsafe booleans.
					There is an array of AtomicBoolean objects, and the get and set use Boolean objects.
					*** Uses builder pattern! You must finish an instance with .build()!
					Builder pattern used so default settings could be handled without several constructors
					F must always have a toString() and equals() method, or you'll have to override some methods
					https://stackoverflow.com/questions/529085/how-to-create-a-generic-array-in-java
				*/

				S[] value_array = null;
				private LogOption log_option = LogOption.NEVER;
				private String key = "default";
				private Class<S> array_class = null;
				private Class<F> value_class = null;
				private F default_value = null;
				private int size = 1;

				State(Class<S> _array_class, Class<F> _value_class)
				{
						array_class = _array_class;
						value_class = _value_class;
				}

				State<S, F> build() throws Exception
				{
						// array_class and value_class MUST HAVE BEEN SET
						if (array_class == null || value_class == null)
						{
								throw new Exception("VehicleState: State.build() used without setting array and value class properly");
						}
						value_array = (S[])(Array.newInstance(array_class, size));
						try
						{
								Class primitive_class;
								// TODO: improve this ugly workaround (Double, Boolean, Long, and Integer not having constructors that make any sense)
								if (default_value instanceof Double)
								{
										primitive_class = Double.TYPE;
								}
								else if (default_value instanceof Boolean)
								{
										primitive_class = Boolean.TYPE;
								}
								else if (default_value instanceof Long)
								{
										primitive_class = Long.TYPE;
								}
								else if (default_value instanceof Integer)
								{
										primitive_class = Integer.TYPE;
								}
								else
								{
										primitive_class = Void.TYPE;
								}
								if (primitive_class != Void.TYPE)
								{
										Constructor value_constructor = value_class.getConstructor(primitive_class);
										Constructor array_constructor = array_class.getConstructor(primitive_class);
										for (int i = 0; i < value_array.length; i++)
										{
												Object constructed_value = value_constructor.newInstance(default_value);
												value_array[i] = (S) array_constructor.newInstance(constructed_value);
										}
								}
								else
								{
										Constructor array_constructor = array_class.getConstructor();
										for (int i = 0; i < value_array.length; i++)
										{
												value_array[i] = (S) array_constructor.newInstance();
										}
								}
						}
						catch (Exception e)
						{
								Log.e(logTag, String.format("State class constructor error: %s", e.getMessage()));
						}
						return this;
				}

				State<S, F> defaultValue(F _default_value)
				{
						default_value = _default_value;
						return this;
				}

				State<S, F> size(int _size)
				{
						size = _size;
						return this;
				}

				State<S, F> logOption(LogOption _log_option)
				{
						log_option = _log_option;
						return this;
				}

				State<S, F> key(String _key)
				{
						key = _key;
						return this;
				}

				abstract F customGet(int index); // the custom implementation of get
				abstract void customSet(int index, F in); // changing value in a map or array

				// NOTE YOU MUST OVERRIDE isEq() IF YOU USE A CUSTOM TYPE!
				boolean isEq(F new_value) { return isEq(0, new_value); }
				boolean isEq(int index, F new_value)
				{
						if (index >= 0 && index < value_array.length)
						{
								return get(index).equals(new_value);
						}
						return false;
				}

				public F get() { return get(0); } // get first element

				public F get(int index) // the interface get(), automatically check for acceptable index
				{
						if (index >= 0 && index < value_array.length)
						{
								return customGet(index);
						}
						return null;
				}

				public void set(int index, F in)
				{
						if (index >= 0 && index < value_array.length)
						{
								switch (log_option)
								{
										case NEVER:
												break;
										case PRINT_ANY_SET:
												Log.d(logTag, String.format("setState %s[%d] = %s", key, index, in.toString()));
												try
												{
														_serverImpl.mLogger.info(new JSONObject().put(key, in.toString()));
												}
												catch (Exception e)
												{
														Log.e(logTag, "VehicleState logging error: " + e.getMessage());
												}
												break;
										case PRINT_WHEN_CHANGED:
												// Log.v(logTag, String.format("setState %s[%d] comparing old and new values", key, index));
												//
												if (!isEq(index, in))
												{
														Log.d(logTag, String.format("setState changed %s[%d] = %s", key, index, in.toString()));
														try
														{
																_serverImpl.mLogger.info(new JSONObject().put(key, in.toString()));
														}
														catch (Exception e)
														{
																Log.e(logTag, "VehicleState logging error: " + e.getMessage());
														}
												}
												break;
										default:
												break;
								}
								customSet(index, in);
						}
				}

				public void set(F in) { set(0, in); } // set first element

				public void setAll(F in)
				{
						for (int i = 0; i < value_array.length; i++)
						{
								set(i, in);
						}
				}
		}

		class BooleanState extends State<AtomicBoolean, Boolean>
		{
				BooleanState() { super(AtomicBoolean.class, Boolean.class); }
				@Override
				public Boolean customGet(int index)
				{
						return value_array[index].get();
				}
				@Override
				public void customSet(int index, Boolean in)
				{
						value_array[index].set(in);
				}
		}

		class IntegerState extends State<AtomicInteger, Integer>
		{
				IntegerState() { super(AtomicInteger.class, Integer.class); }
				@Override
				Integer customGet(int index)
				{
						return value_array[index].get();
				}
				@Override
				void customSet(int index, Integer in)
				{
						value_array[index].set(in);
				}
		}

		class LongState extends State<AtomicLong, Long>
		{
				LongState() { super(AtomicLong.class, Long.class); }
				@Override
				Long customGet(int index)
				{
						return value_array[index].get();
				}
				@Override
				void customSet(int index, Long in)
				{
						value_array[index].set(in);
				}
		}

		class DoubleState extends State<Double, Double>
		{
				DoubleState() { super(Double.class, Double.class); }
				final Object lock = new Object();
				@Override
				public Double customGet(int index)
				{
						synchronized (lock)
						{
								return value_array[index];
						}
				}

				@Override
				public void customSet(int index, Double in)
				{
						synchronized (lock)
						{
								value_array[index] = in;
						}
				}
		}

		class UtmPoseState extends State<UtmPose, UtmPose>
		{
				UtmPoseState() { super(UtmPose.class, UtmPose.class); }
				final Object lock = new Object();
				@Override
				public UtmPose customGet(int index)
				{
						synchronized (lock)
						{
								if (value_array[index] == null)
								{
										Log.w(logTag, "The requested UtmPose is null");
										return null;
								}
								return value_array[index].clone();
						}
				}

				@Override
				public void customSet(int index, UtmPose in)
				{
						synchronized (lock)
						{
								if (in == null)
								{
										Log.w(logTag, "The supplied UtmPose is null.");
										return;
								}
								value_array[index] = in.clone();
						}
				}
		}

		public <F> F get(String state_name)
		{
				if (!state_map.containsKey(state_name))
				{
						Log.e(logTag, String.format("state \"%s\" does not exist", state_name));
						return null;
				}
				Object result = state_map.get(state_name).get();
				if (result == null)
				{
						Log.w(logTag, String.format("state \"%s\" returned null", state_name));
						return null;
				}
				return (F)result;
		}
		public <F> F get(String state_name, int index)
		{
				if (!state_map.containsKey(state_name))
				{
						Log.e(logTag, String.format("state \"%s\" does not exist", state_name));
						return null;
				}
				Object result = state_map.get(state_name).get(index);
				if (result == null)
				{
						Log.w(logTag, String.format("state \"%s\"[%d] returned null", state_name, index));
						return null;
				}
				return (F)result;
		}
		public <F> void set(String state_name, F in)
		{
				if (!state_map.containsKey(state_name))
				{
						Log.e(logTag, String.format("Tried to set \"%s\", which does not exist", state_name));
						return;
				}
				state_map.get(state_name).set(in);
		}
		public <F> void set(String state_name, int index, F in)
		{
				if (!state_map.containsKey(state_name))
				{
						Log.e(logTag, String.format("Tried to set \"%s\", which does not exist", state_name));
						return;
				}
				state_map.get(state_name).set(index, in);
		}

		VehicleState(VehicleServerImpl server)
		{
				_serverImpl = server;

				for (int i = 0; i < jar_available.length; i++)
				{
						jar_available[i] = new AtomicBoolean(true); // all jars initially available
				}

				try
				{
						state_map.put(States.EXAMPLE_STATE.name,
								new State<AtomicBoolean, Boolean>(AtomicBoolean.class, Boolean.class)
								{
										@Override
										Boolean customGet(int index)
										{
												value_array[index].set(!value_array[index].get());
												return value_array[index].get();
										}

										@Override
										void customSet(int index, Boolean in) { }
								}
								.defaultValue(false)
								.key(States.EXAMPLE_STATE.name)
								.build()
						);

						state_map.put(States.EXAMPLE_VALUE.name,
								new State<Double, Double>(Double.class, Double.class)
								{
										Object lock = new Object();

										@Override
										Double customGet(int index)
										{
												synchronized (lock)
												{
														value_array[index] += 1.0;
														return value_array[index];
												}
										}

										@Override
										void customSet(int index, Double in) { }
								}
								.defaultValue(Double.valueOf(0.0))
								.key(States.EXAMPLE_VALUE.name)
								.build()
						);

						state_map.put(States.ALWAYS_FALSE.name,
								new State<Boolean, Boolean>(Boolean.class, Boolean.class)
								{
										@Override
										Boolean customGet(int index)
										{
												return false;
										}

										@Override
										void customSet(int index, Boolean in)
										{
										}
								}
								.defaultValue(Boolean.valueOf(false))
								.key(States.ALWAYS_FALSE.name)
								.build()
						);

						state_map.put(States.ALWAYS_TRUE.name,
								new State<Boolean, Boolean>(Boolean.class, Boolean.class)
								{
										@Override
										Boolean customGet(int index)
										{
												return true;
										}

										@Override
										void customSet(int index, Boolean in)
										{
										}
								}
								.defaultValue(Boolean.valueOf(true))
								.key(States.ALWAYS_TRUE.name)
								.build()
						);

						state_map.put(States.EXAMPLE_ARRAY.name,
								new State<Long, Long>(Long.class, Long.class)
								{
										long counter;
										Object lock = new Object();

										@Override
										Long customGet(int index)
										{
												synchronized (lock)
												{
														value_array[index] = ++counter;
														return value_array[index];
												}
										}

										@Override
										void customSet(int index, Long in) { }
								}
								.size(3)
								.defaultValue(Long.valueOf(0))
								.key(States.EXAMPLE_ARRAY.name)
								.build()
						);


						state_map.put(States.IS_CONNECTED.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.IS_CONNECTED.name)
										.build()
						);
						state_map.put(States.IS_AUTONOMOUS.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.IS_AUTONOMOUS.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);
						state_map.put(States.IS_RUNNING.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.IS_RUNNING.name)
										.build()
						);
						state_map.put(States.HAS_FIRST_AUTONOMY.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.HAS_FIRST_AUTONOMY.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);
						state_map.put(States.HAS_FIRST_GPS.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.HAS_FIRST_GPS.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);
						state_map.put(States.IS_GOING_HOME.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.IS_GOING_HOME.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);
						state_map.put(States.IS_TAKING_SAMPLE.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.IS_TAKING_SAMPLE.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);
						state_map.put(States.RC_OVERRIDE_IS_ON.name,
										new BooleanState()
										.defaultValue(Boolean.valueOf(false))
										.key(States.RC_OVERRIDE_IS_ON.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);


						state_map.put(States.EC.name,
										new DoubleState()
										.defaultValue(Double.valueOf(0.0))
										.key(States.EC.name)
										.build()
						);
						state_map.put(States.T.name,
										new DoubleState()
										.defaultValue(Double.valueOf(0.0))
										.key(States.T.name)
										.build()
						);
						state_map.put(States.DO.name,
										new DoubleState()
										.defaultValue(Double.valueOf(0.0))
										.key(States.DO.name)
										.build()
						);
						state_map.put(States.PH.name,
										new DoubleState()
										.defaultValue(Double.valueOf(0.0))
										.key(States.PH.name)
										.build()
						);
						state_map.put(States.WATER_DEPTH.name,
										new DoubleState()
										.defaultValue(Double.valueOf(0.0))
										.key(States.WATER_DEPTH.name)
										.build()
						);
						state_map.put(States.BATTERY_VOLTAGE.name,
										new DoubleState()
										.defaultValue(Double.valueOf(0.0))
										.key(States.BATTERY_VOLTAGE.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);

						state_map.put(States.ELAPSED_TIME.name,
								new State<AtomicLong, Long>(AtomicLong.class, Long.class)
								{
										long first = System.currentTimeMillis();

										@Override
										Long customGet(int index)
										{
												try
												{
														value_array[index].set(System.currentTimeMillis() - first);
														Log.v(logTag, String.format("retrieved elapsed time = %d ms", value_array[index].get()));
														return value_array[index].get();
												}
												catch (Exception e)
												{
														Log.e(logTag, String.format("Elapsed time customGet() error: %s", e.getMessage()));
														return null;
												}
										}

										@Override
										void customSet(int index, Long in) { }
								}
								.defaultValue(Long.valueOf(0))
								.key(States.ELAPSED_TIME.name)
								.build()
						);

						state_map.put(States.TIME_SINCE_OPERATOR.name,
								new State<AtomicLong, Long>(AtomicLong.class, Long.class)
								{
										@Override
										Long customGet(int index)
										{
												return System.currentTimeMillis() - value_array[index].get();
										}

										@Override
										void customSet(int index, Long in)
										{
												value_array[index].set(System.currentTimeMillis()); // set to now, ignore input argument
										}
								}
								.defaultValue(Long.valueOf(0))
								.key(States.TIME_SINCE_OPERATOR.name)
								.build()
						);

						state_map.put(States.CURRENT_POSE.name,
										new UtmPoseState()
										.defaultValue(new UtmPose())
										.key(States.CURRENT_POSE.name)
										.build()
						);

						state_map.put(States.HOME_POSE.name,
										new UtmPoseState()
										.defaultValue(new UtmPose())
										.key(States.HOME_POSE.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);

						state_map.put(States.FIRST_POSE.name,
										new UtmPoseState()
										.defaultValue(new UtmPose())
										.key(States.FIRST_POSE.name)
										.logOption(LogOption.PRINT_WHEN_CHANGED)
										.build()
						);

						state_map.put(States.NEXT_AVAILABLE_JAR.name,
								new State<AtomicInteger, Integer>(AtomicInteger.class, Integer.class)
								{
										// note how the value_array is totally ignored here.
										// Limitations of the class force the "jar_available" boolean array to be outside
										@Override
										Integer customGet(int index)
										{
												for (int i = 0; i < jar_available.length; i++)
												{
														if (jar_available[i].get()) return Integer.valueOf(i);
												}
												return -1;
										}

										@Override
										void customSet(int index, Integer in) { }
								}
								.key(States.NEXT_AVAILABLE_JAR.name)
								.defaultValue(Integer.valueOf(0))
								.logOption(LogOption.PRINT_WHEN_CHANGED)
								.build()
						);

						state_map.put(States.JARS_AVAILABLE.name,
								new State<AtomicBoolean, Boolean>(AtomicBoolean.class, Boolean.class)
								{
										@Override
										Boolean customGet(int index)
										{
												Long next_available_jar = (Long) state_map.get(States.NEXT_AVAILABLE_JAR.name).get();
												return (next_available_jar >= 0);
										}

										@Override
										void customSet(int index, Boolean in)
										{
												value_array[index].set(in);
										}
								}
								.key(States.JARS_AVAILABLE.name)
								.logOption(LogOption.PRINT_WHEN_CHANGED)
								.build()
						);
				}
				catch (Exception e)
				{
						Log.e(logTag, "VehicleState construction error: " + e.getMessage());
				}
		}

		void usingJar(int i)
		{
				jar_available[i].set(false);
		}
		void resetSampleJars()
		{
				for (int i = 0; i < jar_available.length; i++)
				{
						resetSampleJar(i);
				}
		}
		private void resetSampleJar(int i)
		{
				jar_available[i].set(true);
		}
}
