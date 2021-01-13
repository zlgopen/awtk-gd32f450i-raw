﻿/**
 * File:   fscript_widget.c
 * Author: AWTK Develop Team
 * Brief:  widget functions for fscript
 *
 * Copyright (c) 2020 - 2021  Guangzhou ZHIYUAN Electronics Co.,Ltd.
 *
 */

/**
 * History:
 * ================================================================
 * 2021-01-08 Li XianJing <lixianjing@zlg.cn> created
 *
 */

#include "awtk_global.h"
#include "tkc/fscript.h"
#include "tkc/tokenizer.h"
#include "base/main_loop.h"
#include "base/window.h"
#include "base/locale_info.h"
#include "base/widget_factory.h"
#include "base/window_manager.h"
#include "ui_loader/ui_builder_default.h"

static widget_t* find_target_widget(widget_t* widget, const char* path, uint32_t len) {
  bool_t is_first = TRUE;
  tokenizer_t tokenizer;
  widget_t* iter = widget;
  tokenizer_t* t = NULL;
  return_value_if_fail(widget != NULL && path != NULL, NULL);
  t = tokenizer_init(&tokenizer, path, len, ".");
  return_value_if_fail(t != NULL, NULL);

  while (tokenizer_has_more(t) && iter != NULL) {
    const char* name = tokenizer_next(t);
    if (is_first) {
      if (tk_str_eq(name, STR_PROP_PARENT)) {
        iter = widget->parent;
      } else if (tk_str_eq(name, STR_PROP_WINDOW)) {
        iter = widget_get_window(widget);
      } else if (tk_str_eq(name, STR_PROP_WINDOW_MANAGER)) {
        iter = widget_get_window_manager(widget);
      } else {
        iter = widget_child(iter, name);
      }
      is_first = FALSE;
    } else {
      iter = widget_child(iter, name);
    }
  }
  tokenizer_deinit(t);

  return iter;
}

static widget_t* to_widget(fscript_t* fscript, const value_t* v) {
  if (v->type == VALUE_TYPE_STRING) {
    widget_t* self = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
    const char* path = value_str(v);
    return_value_if_fail(path != NULL, NULL);

    return find_target_widget(self, path, strlen(path));
  } else if (v->type == VALUE_TYPE_POINTER) {
    return WIDGET(value_pointer(v));
  } else {
    return NULL;
  }
}

static ret_t func_tr(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  FSCRIPT_FUNC_CHECK(args->size == 1, RET_BAD_PARAMS);
  value_dup_str(result, locale_info_tr(locale_info(), value_str(args->args)));

  return RET_OK;
}

static ret_t func_window_open(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  widget_t* widget = NULL;
  const char* name = NULL;
  bool_t close_current = FALSE;
  bool_t switch_to_if_exist = FALSE;
  widget_t* wm = window_manager();
  widget_t* self = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
  widget_t* curr_win = widget_get_window(self);
  FSCRIPT_FUNC_CHECK(args->size >= 1, RET_BAD_PARAMS);
  name = value_str(args->args);
  close_current = args->size > 1 ? value_bool(args->args + 1) : FALSE;
  switch_to_if_exist = args->size > 2 ? value_bool(args->args + 2) : FALSE;
  FSCRIPT_FUNC_CHECK(name != NULL, RET_BAD_PARAMS);

  if (switch_to_if_exist) {
    widget_t* widget = widget_child(wm, name);
    if (widget != NULL) {
      window_manager_switch_to(wm, curr_win, widget, close_current);
      value_set_pointer(result, widget);
      return RET_OK;
    }
  }

  if (close_current) {
    widget = window_open_and_close(value_str(args->args), curr_win);
  } else {
    widget = window_open(value_str(args->args));
  }

  value_set_pointer(result, widget);
  return RET_OK;
}

static ret_t func_window_close(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  ret_t ret = RET_OK;

  if (args->size == 1) {
    value_t* v = args->args;
    widget_t* wm = window_manager();
    widget_t* win = widget_child(wm, value_str(v));
    FSCRIPT_FUNC_CHECK(win != NULL, RET_BAD_PARAMS);
    ret = window_manager_close_window_force(wm, win);
  } else {
    widget_t* self = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
    ret = window_close(widget_get_window(self));
  }
  value_set_bool(result, ret == RET_OK);

  return ret;
}

static ret_t widget_set(widget_t* self, const char* path, const value_t* v) {
  widget_t* widget = self;
  const char* prop = strrchr(path, '.');
  if (prop != NULL) {
    widget = find_target_widget(self, path, prop - path);
    prop++;
  } else {
    prop = path;
  }
  return_value_if_fail(widget != NULL, RET_BAD_PARAMS);

  return widget_set_prop(widget, prop, v);
}

static ret_t widget_get(widget_t* self, const char* path, value_t* v) {
  widget_t* widget = self;
  const char* prop = strrchr(path, '.');
  if (prop != NULL) {
    widget = find_target_widget(self, path, prop - path);
    prop++;
  } else {
    prop = path;
  }
  return_value_if_fail(widget != NULL, RET_BAD_PARAMS);

  return widget_get_prop(widget, prop, v);
}

static ret_t my_quit_idle(const timer_info_t* timer) {
  main_loop_t* loop = main_loop();

  loop->app_quited = TRUE;

  return main_loop_quit(loop);
}

static ret_t func_quit(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  timer_add(my_quit_idle, NULL, 0);

  return RET_OK;
}

static ret_t func_back(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  window_manager_back(window_manager());

  return RET_OK;
}

static ret_t func_back_to_home(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  window_manager_back_to_home(window_manager());

  return RET_OK;
}

static ret_t func_widget_lookup(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  widget_t* widget = NULL;
  const char* path = NULL;
  bool_t recursive = FALSE;
  FSCRIPT_FUNC_CHECK(args->size >= 1, RET_BAD_PARAMS);

  if (args->size == 1) {
    widget = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
    path = value_str(args->args);
  } else {
    widget = to_widget(fscript, args->args);
    path = value_str(args->args + 1);
    recursive = args->size > 2 ? value_bool(args->args + 2) : FALSE;
  }
  FSCRIPT_FUNC_CHECK(widget != NULL && path != NULL, RET_BAD_PARAMS);

  if (strchr(path, '.') != NULL) {
    widget = find_target_widget(widget, path, strlen(path));
  } else {
    widget = widget_lookup(widget, path, recursive);
  }
  value_set_pointer(result, widget);

  return RET_OK;
}

static ret_t func_widget_get(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  widget_t* widget = NULL;
  const char* path = NULL;
  FSCRIPT_FUNC_CHECK(args->size >= 1, RET_BAD_PARAMS);

  if (args->size == 1) {
    widget = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
    path = value_str(args->args);
  } else {
    widget = to_widget(fscript, args->args);
    path = value_str(args->args + 1);
  }
  FSCRIPT_FUNC_CHECK(widget != NULL && path != NULL, RET_BAD_PARAMS);

  return widget_get(widget, path, result);
}

static ret_t func_widget_set(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  value_t* v = NULL;
  ret_t ret = RET_OK;
  widget_t* widget = NULL;
  const char* path = NULL;
  FSCRIPT_FUNC_CHECK(args->size >= 2, RET_BAD_PARAMS);

  if (args->size == 2) {
    widget = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
    path = value_str(args->args);
    v = args->args + 1;
  } else {
    widget = to_widget(fscript, args->args);
    path = value_str(args->args + 1);
    v = args->args + 2;
  }
  FSCRIPT_FUNC_CHECK(widget != NULL && path != NULL, RET_BAD_PARAMS);

  ret = widget_set(widget, path, v);
  value_set_bool(result, ret == RET_OK);

  return ret;
}

static ret_t func_widget_create(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  int32_t x = 0;
  int32_t y = 0;
  int32_t w = 0;
  int32_t h = 0;
  const char* type = NULL;
  widget_t* widget = NULL;
  widget_t* parent = NULL;
  FSCRIPT_FUNC_CHECK(args->size == 6, RET_BAD_PARAMS);
  type = value_str(args->args);
  parent = to_widget(fscript, args->args + 1);
  return_value_if_fail(type != NULL && parent != NULL, RET_BAD_PARAMS);
  x = value_int(args->args + 2);
  y = value_int(args->args + 3);
  w = value_int(args->args + 4);
  h = value_int(args->args + 5);
  widget = widget_factory_create_widget(widget_factory(), type, parent, x, y, w, h);
  value_set_pointer(result, widget);

  return RET_OK;
}

static ret_t func_widget_destroy(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  widget_t* widget = NULL;
  FSCRIPT_FUNC_CHECK(args->size == 1, RET_BAD_PARAMS);
  widget = to_widget(fscript, args->args);
  FSCRIPT_FUNC_CHECK(widget != NULL, RET_BAD_PARAMS);

  value_set_bool(result, widget_destroy(widget) == RET_OK);

  return RET_OK;
}

#define STR_PROP_TIMER_ID "_timer_id_"

static ret_t widget_on_timer(const timer_info_t* info) {
  widget_t* widget = WIDGET(info->ctx);
  ret_t ret = widget_dispatch_simple_event(widget, EVT_TIMER);
  ret = ret == RET_REMOVE ? RET_REMOVE : RET_REPEAT;
  if (ret == RET_REMOVE) {
    widget_set_prop_int(widget, STR_PROP_TIMER_ID, TK_INVALID_ID);
  }
  return ret;
}

static ret_t func_widget_add_timer(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  uint32_t id = 0;
  uint32_t duration = 0;
  widget_t* self = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
  FSCRIPT_FUNC_CHECK(args->size == 1, RET_BAD_PARAMS);
  duration = value_uint32(args->args);
  FSCRIPT_FUNC_CHECK(self != NULL && duration > 0, RET_BAD_PARAMS);

  id = widget_get_prop_int(self, STR_PROP_TIMER_ID, TK_INVALID_ID);
  if (id != TK_INVALID_ID) {
    timer_remove(id);
    log_debug("timer exist, remove it.\n");
  }

  id = widget_add_timer(self, widget_on_timer, duration);
  value_set_uint32(result, id);
  widget_set_prop_int(self, STR_PROP_TIMER_ID, id);

  return RET_OK;
}

static ret_t func_widget_remove_timer(fscript_t* fscript, fscript_args_t* args, value_t* result) {
  uint32_t id = 0;
  widget_t* widget = WIDGET(object_get_prop_pointer(fscript->obj, STR_PROP_SELF));
  FSCRIPT_FUNC_CHECK(widget != NULL, RET_BAD_PARAMS);
  if (args->size > 0) {
    widget = to_widget(fscript, args->args);
  }
  FSCRIPT_FUNC_CHECK(widget != NULL, RET_BAD_PARAMS);

  id = widget_get_prop_int(widget, STR_PROP_TIMER_ID, TK_INVALID_ID);
  if (id != TK_INVALID_ID) {
    timer_remove(id);
    value_set_bool(result, TRUE);
    widget_set_prop_int(widget, STR_PROP_TIMER_ID, TK_INVALID_ID);
  } else {
    value_set_bool(result, FALSE);
    log_debug("not found timer\n");
  }

  return RET_OK;
}

ret_t fscript_widget_register(void) {
  ENSURE(fscript_register_func("open", func_window_open) == RET_OK);
  ENSURE(fscript_register_func("close", func_window_close) == RET_OK);
  ENSURE(fscript_register_func("back", func_back) == RET_OK);
  ENSURE(fscript_register_func("back_to_home", func_back_to_home) == RET_OK);
  ENSURE(fscript_register_func("quit", func_quit) == RET_OK);
  ENSURE(fscript_register_func("tr", func_tr) == RET_OK);

  ENSURE(fscript_register_func("widget_lookup", func_widget_lookup) == RET_OK);
  ENSURE(fscript_register_func("widget_get", func_widget_get) == RET_OK);
  ENSURE(fscript_register_func("widget_set", func_widget_set) == RET_OK);
  ENSURE(fscript_register_func("widget_create", func_widget_create) == RET_OK);
  ENSURE(fscript_register_func("widget_destroy", func_widget_destroy) == RET_OK);
  ENSURE(fscript_register_func("start_timer", func_widget_add_timer) == RET_OK);
  ENSURE(fscript_register_func("stop_timer", func_widget_remove_timer) == RET_OK);

  return RET_OK;
}
