#include "serial_encoder.h"


size_t Encode_GY85_Data(GY85_Def_t* obj, char *buffer, size_t buffer_size) {
    mpack_writer_t writer;
    mpack_writer_init(&writer, buffer, buffer_size);

    // Start the map with 4 key-value pairs
    mpack_start_map(&writer, 4);

	mpack_write_int(&writer, ACCEL_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->accelX);
	mpack_write_float(&writer, obj->accelY);
	mpack_write_float(&writer, obj->accelZ);

	mpack_finish_array(&writer);

	mpack_write_int(&writer, COMPASS_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_int(&writer, obj->compassX);
	mpack_write_int(&writer, obj->compassY);
	mpack_write_int(&writer, obj->compassZ);

	mpack_finish_array(&writer);
	
	mpack_write_int(&writer, GYRO_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->gyroX);
	mpack_write_float(&writer, obj->gyroY);
	mpack_write_float(&writer, obj->gyroZ);

	mpack_finish_array(&writer);
	
	mpack_write_int(&writer, TEMP_ID);  // write the key
	mpack_write_float(&writer, obj->gyroTemp);
	
    // Finish the map
    mpack_finish_map(&writer);

    if (mpack_writer_destroy(&writer) != mpack_ok) {
        printf("Encoding error\n");
    }
	return mpack_writer_buffer_used(&writer);
}


void Decode_GY85_Data(GY85_Def_t* obj, char *buffer, size_t buffer_size) {
    mpack_reader_t reader;
    mpack_reader_init_data(&reader, buffer, buffer_size);
	
///////////////////////////////////////////////////////////////////
    // Expect the map with 4 key-value pairs
    mpack_expect_map_max(&reader, 4);
///////////////////////////////////////////////////////////////////
	int key=-1;

	for(int i=0; i<4; i++)
	{
		// Read the Accelaration key
		key = mpack_expect_int(&reader);
		switch(key)
		{
			case ACCEL_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->accelX = mpack_expect_float(&reader);
				obj->accelY = mpack_expect_float(&reader);
				obj->accelZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
				break;
			///////////////////////////////////////////////////////////////////		
			case COMPASS_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->compassX = mpack_expect_int(&reader);
				obj->compassY = mpack_expect_int(&reader);
				obj->compassZ = mpack_expect_int(&reader);
				mpack_done_array(&reader);
			break;
			///////////////////////////////////////////////////////////////////
			case GYRO_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->gyroX = mpack_expect_float(&reader);
				obj->gyroY = mpack_expect_float(&reader);
				obj->gyroZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
			break;
			///////////////////////////////////////////////////////////////////
			case TEMP_ID :
				obj->gyroTemp= mpack_expect_float(&reader);
				///////////////////////////////////////////////////////////////////
				break;
			default:
				printf("Key mismatch....\n");
		}
	}
	mpack_done_map(&reader);
	
    if (mpack_reader_destroy(&reader) != mpack_ok) {
        printf("Decoding error\n");
    }
}

/***********************************************************************************************/


size_t Encode_HMC6343_Data(HMC6343_Def_t* obj, char *buffer, size_t buffer_size) {
    mpack_writer_t writer;
    mpack_writer_init(&writer, buffer, buffer_size);

    // Start the map with 3 key-value pairs
    mpack_start_map(&writer, 3);

	mpack_write_int(&writer, ACCEL_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_int(&writer, obj->accelX);
	mpack_write_int(&writer, obj->accelY);
	mpack_write_int(&writer, obj->accelZ);

	mpack_finish_array(&writer);

	mpack_write_int(&writer, MAG_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_int(&writer, obj->magX);
	mpack_write_int(&writer, obj->magY);
	mpack_write_int(&writer, obj->magZ);

	mpack_finish_array(&writer);
	
	mpack_write_int(&writer, TILT_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_int(&writer, obj->heading);
	mpack_write_int(&writer, obj->pitch);
	mpack_write_int(&writer, obj->roll);

	mpack_finish_array(&writer);
	
    // Finish the map
    mpack_finish_map(&writer);

    if (mpack_writer_destroy(&writer) != mpack_ok) {
        printf("Encoding error\n");
    }
	return mpack_writer_buffer_used(&writer);
}


void Decode_HMC6343_Data(HMC6343_Def_t* obj, char *buffer, size_t buffer_size) {
    mpack_reader_t reader;
    mpack_reader_init_data(&reader, buffer, buffer_size);
	
///////////////////////////////////////////////////////////////////
    // Expect the map with 3 key-value pairs
    mpack_expect_map_max(&reader, 3);
///////////////////////////////////////////////////////////////////
	int key=-1;

	for(int i=0; i<3; i++)
	{
		// Read the Accelaration key
		key = mpack_expect_int(&reader);
		switch(key)
		{
			case ACCEL_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->accelX = mpack_expect_int(&reader);
				obj->accelY = mpack_expect_int(&reader);
				obj->accelZ = mpack_expect_int(&reader);
				mpack_done_array(&reader);
				break;
			///////////////////////////////////////////////////////////////////		
			case MAG_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->magX = mpack_expect_int(&reader);
				obj->magY = mpack_expect_int(&reader);
				obj->magZ = mpack_expect_int(&reader);
				mpack_done_array(&reader);
			break;
			///////////////////////////////////////////////////////////////////
			case TILT_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->heading = mpack_expect_int(&reader);
				obj->pitch = mpack_expect_int(&reader);
				obj->roll = mpack_expect_int(&reader);
				mpack_done_array(&reader);
			break;
			///////////////////////////////////////////////////////////////////
			default:
				printf("Key mismatch....\n");
		}
	}
	mpack_done_map(&reader);
	
    if (mpack_reader_destroy(&reader) != mpack_ok) {
        printf("Decoding error\n");
    }
}

/***********************************************************************************************/
size_t Encode_MPU9250_Data(MPU9250_Def_t* obj, char *buffer, size_t buffer_size) {

    mpack_writer_t writer;
    mpack_writer_init(&writer, buffer, buffer_size);

    // Start the map with 4 key-value pairs
    mpack_start_map(&writer, 4);

	mpack_write_int(&writer, ACCEL_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->accelX);
	mpack_write_float(&writer, obj->accelY);
	mpack_write_float(&writer, obj->accelZ);

	mpack_finish_array(&writer);

	mpack_write_int(&writer, MAG_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->magX);
	mpack_write_float(&writer, obj->magY);
	mpack_write_float(&writer, obj->magZ);

	mpack_finish_array(&writer);
	
	mpack_write_int(&writer, GYRO_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->gyroX);
	mpack_write_float(&writer, obj->gyroY);
	mpack_write_float(&writer, obj->gyroZ);

	mpack_finish_array(&writer);
	
	mpack_write_int(&writer, TEMP_ID);  // write the key
	mpack_write_float(&writer, obj->temperature);

	//mpack_write_int(&writer, RESULTANTG_ID);  // write the key
	//mpack_write_float(&writer, obj->resultantG);		


    // Finish the map
    mpack_finish_map(&writer);

    if (mpack_writer_destroy(&writer) != mpack_ok) {
        printf("Encoding error\n");
    }
	return mpack_writer_buffer_used(&writer);
}

void Decode_MPU9250_Data(MPU9250_Def_t* obj, char *buffer, size_t buffer_size) {
	mpack_reader_t reader;
    mpack_reader_init_data(&reader, buffer, buffer_size);
	
///////////////////////////////////////////////////////////////////
    // Expect the map with 3 key-value pairs
    mpack_expect_map_max(&reader, 4);
///////////////////////////////////////////////////////////////////
	int key=-1;

	for(int i=0; i<4; i++)
	{
		// Read the Accelaration key
		key = mpack_expect_int(&reader);
		switch(key)
		{
			case ACCEL_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->accelX = mpack_expect_float(&reader);
				obj->accelY = mpack_expect_float(&reader);
				obj->accelZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
				break;
			///////////////////////////////////////////////////////////////////		
			case MAG_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->magX = mpack_expect_float(&reader);
				obj->magY = mpack_expect_float(&reader);
				obj->magZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
			break;
			///////////////////////////////////////////////////////////////////
			case GYRO_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->gyroX = mpack_expect_float(&reader);
				obj->gyroY = mpack_expect_float(&reader);
				obj->gyroZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
			break;
			case TEMP_ID :
				obj->temperature= mpack_expect_float(&reader);
			///////////////////////////////////////////////////////////////////
			break;
			case RESULTANTG_ID :
				obj->resultantG= mpack_expect_float(&reader);
			///////////////////////////////////////////////////////////////////
			default:
				printf("Key mismatch....\n");
		}
	}
	mpack_done_map(&reader);
	
    if (mpack_reader_destroy(&reader) != mpack_ok) {
        printf("Decoding error\n");
    }

}

/***********************************************************************************************/
size_t Encode_SFE_Data(sfe_io_t* obj, char *buffer, size_t buffer_size) {

    mpack_writer_t writer;
    mpack_writer_init(&writer, buffer, buffer_size);

    // Start the map with 3 key-value pairs
    mpack_start_map(&writer, 3);

	mpack_write_int(&writer, ACCEL_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->accelX);
	mpack_write_float(&writer, obj->accelY);
	mpack_write_float(&writer, obj->accelZ);

	mpack_finish_array(&writer);

	mpack_write_int(&writer, MAG_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->magX);
	mpack_write_float(&writer, obj->magY);
	mpack_write_float(&writer, obj->magZ);

	mpack_finish_array(&writer);
	
	mpack_write_int(&writer, GYRO_ID);  // write the key
	mpack_start_array(&writer, 3);  // start the inner array

	// Write the elements of the inner array
	mpack_write_float(&writer, obj->gyroX);
	mpack_write_float(&writer, obj->gyroY);
	mpack_write_float(&writer, obj->gyroZ);

	mpack_finish_array(&writer);	


    // Finish the map
    mpack_finish_map(&writer);

    if (mpack_writer_destroy(&writer) != mpack_ok) {
        printf("Encoding error\n");
    }
	return mpack_writer_buffer_used(&writer);
}

void Decode_SFE_Data(sfe_io_t* obj, char *buffer, size_t buffer_size) {
	mpack_reader_t reader;
    mpack_reader_init_data(&reader, buffer, buffer_size);
	
///////////////////////////////////////////////////////////////////
    // Expect the map with 3 key-value pairs
    mpack_expect_map_max(&reader, 3);
///////////////////////////////////////////////////////////////////
	int key=-1;

	for(int i=0; i<3; i++)
	{
		// Read the Accelaration key
		key = mpack_expect_int(&reader);
		switch(key)
		{
			case ACCEL_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->accelX = mpack_expect_float(&reader);
				obj->accelY = mpack_expect_float(&reader);
				obj->accelZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
				break;
			///////////////////////////////////////////////////////////////////		
			case MAG_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->magX = mpack_expect_float(&reader);
				obj->magY = mpack_expect_float(&reader);
				obj->magZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
			break;
			///////////////////////////////////////////////////////////////////
			case GYRO_ID :
				// Expect and read the inner array
				mpack_expect_array_max(&reader, 3);
				obj->gyroX = mpack_expect_float(&reader);
				obj->gyroY = mpack_expect_float(&reader);
				obj->gyroZ = mpack_expect_float(&reader);
				mpack_done_array(&reader);
			break;
			///////////////////////////////////////////////////////////////////
			default:
				printf("Key mismatch....\n");
		}
	}
	mpack_done_map(&reader);
	
    if (mpack_reader_destroy(&reader) != mpack_ok) {
        printf("Decoding error\n");
    }

}

