﻿/**
 * File:   object_date_time.c
 * Author: AWTK Develop Team
 * Brief:  wrap typed array to an object.
 *
 * Copyright (c) 2020 - 2021  Guangzhou ZHIYUAN Electronics Co.,Ltd.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY { without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * License file for more details.
 *
 */

/**
 * History:
 * ================================================================
 * 2021-01-03 Li XianJing <xianjimli@hotmail.com> created
 *
 */

#include "tkc/mem.h"
#include "tkc/value.h"
#include "tkc/utils.h"
#include "object_date_time.h"

static ret_t object_date_time_on_destroy(object_t* obj) {
  object_date_time_t* o = OBJECT_DATE_TIME(obj);
  return_value_if_fail(o != NULL && o->dt != NULL, RET_BAD_PARAMS);

  date_time_destroy(o->dt);
  o->dt = NULL;

  return RET_OK;
}

static ret_t object_date_time_set_prop(object_t* obj, const char* name, const value_t* v) {
  ret_t ret = RET_OK;
  object_date_time_t* o = OBJECT_DATE_TIME(obj);
  return_value_if_fail(o != NULL && o->dt != NULL, RET_BAD_PARAMS);

  switch(*name) {
    case 'y': {
      date_time_set_year(o->dt, value_uint32(v));
      break;
    }
    case 'm': {
      if(name[1] == 'i') {
        date_time_set_minute(o->dt, value_uint32(v));
      } else {
        date_time_set_month(o->dt, value_uint32(v));
      }
      break;
    }
    case 'd': {
      date_time_set_day(o->dt, value_uint32(v));
      break;
    }
    case 'h': {
      date_time_set_hour(o->dt, value_uint32(v));
      break;
    }
    case 's': {
      date_time_set_second(o->dt, value_uint32(v));
      break;
    }
    default: {
      ret = RET_NOT_FOUND;
    }
  }

  return ret;
}

static ret_t object_date_time_get_prop(object_t* obj, const char* name, value_t* v) {
  ret_t ret = RET_OK;
  object_date_time_t* o = OBJECT_DATE_TIME(obj);
  return_value_if_fail(o != NULL && o->dt != NULL, RET_BAD_PARAMS);

  switch(*name) {
    case 'y': {
      value_set_uint32(v, o->dt->year);
      break;
    }
    case 'm': {
      if(name[1] == 'i') {
        value_set_uint32(v, o->dt->minute);
      } else {
        value_set_uint32(v, o->dt->month);
      }
      break;
    }
    case 'd': {
      value_set_uint32(v, o->dt->day);
      break;
    }
    case 'h': {
      value_set_uint32(v, o->dt->hour);
      break;
    }
    case 's': {
      value_set_uint32(v, o->dt->second);
      break;
    }
    case 'w': {
      value_set_uint32(v, o->dt->wday);
      break;
    }
    default: {
      ret = RET_NOT_FOUND;
    }
  }

  return ret;
}

static const object_vtable_t s_object_date_time_vtable = {
    .type = "object_date_time",
    .desc = "object_date_time",
    .size = sizeof(object_date_time_t),
    .is_collection = FALSE,
    .on_destroy = object_date_time_on_destroy,
    .get_prop = object_date_time_get_prop,
    .set_prop = object_date_time_set_prop};

object_t* object_date_time_create(void) {
  object_t* o = NULL;
  object_date_time_t* wrapper = NULL;
  o = object_create(&s_object_date_time_vtable);
  return_value_if_fail(o != NULL, NULL);

  wrapper = OBJECT_DATE_TIME(o);
  return_value_if_fail(wrapper != NULL, NULL);

  wrapper->dt = date_time_create();
  if (wrapper->dt == NULL) {
    OBJECT_UNREF(o);
  }

  return o;
}

object_date_time_t* object_date_time_cast(object_t* obj) {
  return_value_if_fail(obj != NULL && obj->vt == &s_object_date_time_vtable, NULL);

  return (object_date_time_t*)(obj);
}
